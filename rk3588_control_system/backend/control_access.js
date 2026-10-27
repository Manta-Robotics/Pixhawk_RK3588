import { createHash, randomBytes, randomInt, timingSafeEqual } from 'node:crypto';
import fs from 'node:fs';
import path from 'node:path';

const COOKIE = 'manta_session';
const digest = value => createHash('sha256').update(String(value)).digest();
const clientPattern = /^[a-zA-Z0-9_-]{16,80}$/;

export function loadControlPin(projectRoot, configured = process.env.MANTA_CONTROL_PIN) {
  if (configured !== undefined) {
    if (!/^\d{6}$/.test(configured)) throw new Error('MANTA_CONTROL_PIN must contain exactly six digits');
    return configured;
  }
  const directory = path.join(projectRoot, 'data');
  const filename = path.join(directory, 'control-pin');
  fs.mkdirSync(directory, { recursive: true, mode: 0o700 });
  try {
    fs.writeFileSync(filename, `${String(randomInt(1000000)).padStart(6, '0')}\n`, { flag: 'wx', mode: 0o600 });
  } catch (error) { if (error.code !== 'EEXIST') throw error; }
  const pin = fs.readFileSync(filename, 'utf8').trim();
  if (!/^\d{6}$/.test(pin)) throw new Error('Invalid private data/control-pin file');
  if (process.platform !== 'win32') fs.chmodSync(filename, 0o600);
  console.log('[Access] Control PIN is stored in the private data/control-pin file (value not logged)');
  return pin;
}

export function createControlAccess({ pin, now = Date.now, lifetimeMs = 8 * 60 * 60 * 1000,
  onControlLost = () => false, onChange = () => {} } = {}) {
  if (!/^\d{6}$/.test(pin || '')) throw new Error('A six-digit control PIN is required');
  const pinHash = digest(pin), sessions = new Map(), attempts = new Map(), sockets = new Map();
  let owner = null, revision = 0;
  function notify() { revision++; onChange(); }
  function valid(session) { return Boolean(session && session.expiresAt > now() && sessions.get(session.id) === session); }
  function hasChannel(session, client) { return [...sockets.values()].some(entry => entry.session.id === session.id && entry.client === client && entry.socket.connected); }
  const clientId = request => String(request.headers?.['x-manta-client'] || '');
  const cookieToken = headers => {
    const cookie = String(headers?.cookie || '').split(';').map(part => part.trim()).find(part => part.startsWith(`${COOKIE}=`));
    return cookie ? cookie.slice(COOKIE.length + 1) : '';
  };
  const lookup = headers => {
    const token = cookieToken(headers);
    if (!/^[a-f0-9]{64}$/.test(token)) return null;
    const key = digest(token).toString('hex'), session = sessions.get(key);
    if (!session || session.expiresAt <= now()) return null;
    return session;
  };
  function owns(session, client) { return Boolean(valid(session) && owner && owner.session === session.id && owner.client === client); }
  function clearOwner(reason) {
    if (!owner) return;
    owner = null;
    onControlLost(reason);
    notify();
  }
  function clearDisconnectedOwner(reason) {
    if (!owner) return;
    const session = sessions.get(owner.session);
    if (session && hasChannel(session, owner.client)) return;
    clearOwner(reason);
  }
  function prune() {
    for (const [key, session] of sessions) {
      if (session.expiresAt > now()) continue;
      sessions.delete(key);
      if (owner?.session === session.id) clearOwner('session_expired');
      for (const entry of sockets.values()) if (entry.session.id === session.id) entry.socket.disconnect(true);
    }
    for (const [key, attempt] of attempts) if (now() >= attempt.until) attempts.delete(key);
  }
  function state(session, client) {
    return { revision, authenticated: valid(session), canControl: owns(session, client),
      ownerPresent: Boolean(owner), expiresAt: session?.expiresAt || null };
  }
  function sessionFor(request) { prune(); return lookup(request.headers); }
  function deny(response, status, message) { return response.status(status).json({ success: false, message }); }
  function requireSession(request, response, next) {
    const session = sessionFor(request);
    if (!session) return deny(response, 401, 'Pair with the device before accessing its API');
    request.controlSession = session;
    response.setHeader('Cache-Control', 'no-store');
    next();
  }
  function middleware(request, response, next) {
    requireSession(request, response, () => {
      if (!['GET', 'HEAD', 'OPTIONS'].includes(request.method)
          && request.path !== '/emergency/stop'
          && (!owns(request.controlSession, clientId(request)) || !hasChannel(request.controlSession, clientId(request)))) {
        return deny(response, 403, 'This page does not own control of the device');
      }
      next();
    });
  }
  function installRoutes(app) {
    app.get('/api/auth/status', (request, response) => {
      response.setHeader('Cache-Control', 'no-store');
      response.json({ success: true, controlSession: state(sessionFor(request), clientId(request)) });
    });
    app.post('/api/auth/pair', (request, response) => {
      prune();
      const address = String(request.socket?.remoteAddress || 'unknown');
      const attempt = attempts.get(address);
      if (attempt?.failures >= 5) {
        response.setHeader('Retry-After', String(Math.max(1, Math.ceil((attempt.until - now()) / 1000))));
        return deny(response, 429, 'Too many incorrect pairing attempts; try again later');
      }
      const candidate = String(request.body?.pin || '');
      if (!/^\d{6}$/.test(candidate) || !timingSafeEqual(digest(candidate), pinHash)) {
        if (!attempts.has(address) && attempts.size >= 4096) return deny(response, 429, 'Pairing is temporarily busy');
        attempts.set(address, { failures: (attempt?.failures || 0) + 1, until: attempt?.until || now() + 15 * 60 * 1000 });
        return deny(response, 401, 'Incorrect pairing code');
      }
      attempts.delete(address);
      const existing = lookup(request.headers);
      if (existing) return response.json({ success: true, controlSession: state(existing, clientId(request)) });
      if (sessions.size >= 1024) return deny(response, 429, 'Too many active device sessions');
      const token = randomBytes(32).toString('hex'), id = digest(token).toString('hex');
      const session = { id, expiresAt: now() + lifetimeMs };
      sessions.set(id, session);
      response.setHeader('Cache-Control', 'no-store');
      response.setHeader('Set-Cookie', `${COOKIE}=${token}; Path=/; HttpOnly; SameSite=Strict; Max-Age=${Math.floor(lifetimeMs / 1000)}${request.secure ? '; Secure' : ''}`);
      response.json({ success: true, controlSession: state(session, clientId(request)) });
    });
    app.post('/api/control/claim', requireSession, (request, response) => {
      const client = clientId(request);
      if (!clientPattern.test(client)) return deny(response, 400, 'A valid control-page identifier is required');
      if (!hasChannel(request.controlSession, client)) return deny(response, 409, 'Open the authenticated realtime channel before claiming control');
      if (owner && !owns(request.controlSession, client)) return deny(response, 409, 'Another page already owns control');
      owner = { session: request.controlSession.id, client };
      notify();
      response.json({ success: true, controlSession: state(request.controlSession, client) });
    });
    app.post('/api/control/release', requireSession, (request, response) => {
      if (owner && !owns(request.controlSession, clientId(request))) return deny(response, 403, 'Only the control page can release control');
      if (owner) clearOwner('operator_release');
      response.json({ success: true, controlSession: state(request.controlSession, clientId(request)) });
    });
    app.post('/api/auth/logout', requireSession, (request, response) => {
      if (owner?.session === request.controlSession.id) clearOwner('operator_logout');
      sessions.delete(request.controlSession.id);
      response.setHeader('Set-Cookie', `${COOKIE}=; Path=/; HttpOnly; SameSite=Strict; Max-Age=0${request.secure ? '; Secure' : ''}`);
      for (const entry of [...sockets.values()]) if (entry.session.id === request.controlSession.id) entry.socket.disconnect(true);
      response.json({ success: true });
    });
  }
  function authenticateSocket(socket, next) {
    prune();
    const session = lookup(socket.handshake.headers);
    if (!session) return next(new Error('Pair with the device before opening a control session'));
    const client = String(socket.handshake.auth?.clientId || '');
    socket.data.controlSession = session;
    socket.data.controlClient = client;
    sockets.set(socket.id, { socket, session, client });
    next();
  }
  function authorizeSocket(socket, ownerRequired = true) {
    const session = sessionFor({ headers: socket.handshake.headers });
    if (!session || (ownerRequired && !owns(session, socket.data.controlClient))) {
      socket.emit('error_message', { message: session ? 'This page does not own control of the device' : 'Device session expired; pair again' });
      return false;
    }
    return true;
  }
  function disconnectSocket(socket) {
    const entry = sockets.get(socket.id);
    sockets.delete(socket.id);
    if (!entry || !owns(entry.session, entry.client)) return;
    const samePageStillConnected = [...sockets.values()].some(other => other.session.id === entry.session.id && other.client === entry.client);
    if (samePageStillConnected) return;
    // The callback may retain the owner's lease for an explicitly started
    // autonomous follow run. Manual output is always neutralized on link loss.
    const retain = onControlLost('transport_lost') === true;
    if (!retain) owner = null;
    notify();
  }
  const timer = setInterval(prune, 1000);
  timer.unref();
  return { installRoutes, middleware, authenticateSocket, authorizeSocket, disconnectSocket,
    state, sessionFor, owns, clearOwner, clearDisconnectedOwner, close: () => clearInterval(timer) };
}
