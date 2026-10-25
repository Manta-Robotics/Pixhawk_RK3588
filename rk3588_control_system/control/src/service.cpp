#include "manta/clock_export.hpp"
#include "manta/config_validation.hpp"
#include "manta/control_runtime.hpp"
#include "manta/mavlink_core.hpp"
#include <arpa/inet.h>
#include <chrono>
#include <csignal>
#include <deque>
#include <fcntl.h>
#include <fstream>
#include <iostream>
#include <poll.h>
#include <sys/file.h>
#include <sys/socket.h>
#include <sys/stat.h>
#include <termios.h>
#include <unistd.h>

namespace {
using manta::Json;
volatile std::sig_atomic_t stopping = 0;
void stop(int) {
    stopping = 1;
}
double monotonic_ms() {
    return std::chrono::duration<double, std::milli>(
               std::chrono::steady_clock::now().time_since_epoch())
        .count();
}
double system_wall_ms() {
    return std::floor(std::chrono::duration<double, std::milli>(
                          std::chrono::system_clock::now().time_since_epoch())
                          .count());
}
double wall_ms() {
    // Sensor and controller timestamps share a monotonic clock, expressed in
    // epoch milliseconds for the existing Web contract. NTP/time-zone changes
    // must not turn a fresh sample into a future sample or rewind a lease.
    static const double epoch = std::chrono::duration<double, std::milli>(
                                    std::chrono::system_clock::now().time_since_epoch())
                                    .count();
    static const double origin = monotonic_ms();
    return std::floor(epoch + monotonic_ms() - origin);
}
struct Fd {
    int value = -1;
    Fd() = default;
    explicit Fd(int v) : value(v) {}
    Fd(const Fd &) = delete;
    Fd &operator=(const Fd &) = delete;
    Fd(Fd &&other) noexcept : value(other.value) { other.value = -1; }
    Fd &operator=(Fd &&other) noexcept {
        if (this != &other) {
            close();
            value = other.value;
            other.value = -1;
        }
        return *this;
    }
    ~Fd() { close(); }
    void close() {
        if (value >= 0)
            ::close(value);
        value = -1;
    }
};
sockaddr_in address(const std::string &host, int port) {
    sockaddr_in out{};
    out.sin_family = AF_INET;
    out.sin_port = htons(port);
    if (port < 1 || port > 65535 || inet_pton(AF_INET, host.c_str(), &out.sin_addr) != 1)
        throw std::invalid_argument("invalid IPv4 endpoint");
    return out;
}
bool loopback(const sockaddr_in &endpoint) {
    return (ntohl(endpoint.sin_addr.s_addr) >> 24) == 127;
}
Fd socket_fd(int type) {
    Fd out(socket(AF_INET, type | SOCK_NONBLOCK | SOCK_CLOEXEC, 0));
    if (out.value < 0)
        throw std::runtime_error("socket: " + std::string(strerror(errno)));
    return out;
}
speed_t baud(int value) {
    switch (value) {
    case 9600:
        return B9600;
    case 19200:
        return B19200;
    case 38400:
        return B38400;
    case 57600:
        return B57600;
    case 115200:
        return B115200;
    case 230400:
        return B230400;
    case 460800:
        return B460800;
    case 921600:
        return B921600;
    default:
        throw std::invalid_argument("unsupported serial baud rate");
    }
}
Fd serial(const std::string &path, int rate) {
    Fd out(open(path.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK | O_CLOEXEC));
    if (out.value < 0)
        throw std::runtime_error("open serial: " + std::string(strerror(errno)));
    if (flock(out.value, LOCK_EX | LOCK_NB) < 0)
        throw std::runtime_error("serial port is already owned by another control service");
    termios options{};
    if (tcgetattr(out.value, &options) < 0)
        throw std::runtime_error("tcgetattr failed");
    cfmakeraw(&options);
    options.c_cflag |= CLOCAL | CREAD;
    options.c_cflag &= ~CRTSCTS;
    options.c_cc[VMIN] = 0;
    options.c_cc[VTIME] = 0;
    const auto speed = baud(rate);
    cfsetispeed(&options, speed);
    cfsetospeed(&options, speed);
    if (tcsetattr(out.value, TCSANOW, &options) < 0)
        throw std::runtime_error("tcsetattr failed");
    return out;
}
struct Client {
    Fd fd;
    std::vector<uint8_t> pending;
};
} // namespace

int main(int argc, char **argv) {
    try {
        std::string path = "config/system.config.json", simulator;
        bool check = false;
        if (const char *env = getenv("MANTA_CONFIG_FILE"))
            path = env;
        else if (const char *env = getenv("MANTA_CONFIG_PATH"))
            path = env;
        for (int i = 1; i < argc; ++i) {
            const std::string arg = argv[i];
            if (arg == "--config" && i + 1 < argc)
                path = argv[++i];
            else if (arg == "--simulation-udp" && i + 1 < argc)
                simulator = argv[++i];
            else if (arg == "--check-config")
                check = true;
            else if (arg == "--help") {
                std::cout << "manta-control --config FILE [--check-config] [--simulation-udp "
                             "127.0.0.1:PORT]\n";
                return 0;
            } else
                throw std::invalid_argument("unknown or incomplete argument: " + arg);
        }
        std::ifstream input(path);
        if (!input)
            throw std::runtime_error("cannot read config: " + path);
        const auto config = Json::parse(input);
        if (!config.is_object())
            throw std::invalid_argument("config must be an object");
        const auto command_address = address(config.value("bridge_host", std::string("127.0.0.1")),
                                             config.value("bridge_command_port", 14551));
        const auto telemetry_address =
            address("127.0.0.1", config.value("bridge_telemetry_port", 14552));
        if (!loopback(command_address))
            throw std::invalid_argument("control commands must bind to loopback");
        if (command_address.sin_port == telemetry_address.sin_port)
            throw std::invalid_argument("command and telemetry ports must differ");
        const auto serial_path = config.value("serial_port", std::string("/dev/ttyUSB0"));
        const int baudrate = config.value("baud_rate", 57600);
        baud(baudrate);
        const double lease = manta::number(config, "rover_command_timeout_ms", 500);
        if (lease < 100 || lease > 2000)
            throw std::invalid_argument("rover command lease must be 100..2000 ms");
        const bool simulation =
            getenv("MANTA_SIMULATION") && std::string(getenv("MANTA_SIMULATION")) == "1";
        sockaddr_in peer{};
        if (!simulator.empty()) {
            const auto colon = simulator.find(':');
            if (colon == std::string::npos)
                throw std::invalid_argument("simulation endpoint requires HOST:PORT");
            peer = address(simulator.substr(0, colon), std::stoi(simulator.substr(colon + 1)));
            if (!simulation || !loopback(peer) || serial_path != "/dev/manta-sitl-virtual")
                throw std::invalid_argument("simulation requires MANTA_SIMULATION=1, loopback peer "
                                            "and virtual serial config");
            if (ntohs(command_address.sin_port) != 15551 ||
                ntohs(telemetry_address.sin_port) != 15552)
                throw std::invalid_argument("simulation requires isolated ports 15551/15552");
        } else if (simulation || serial_path == "/dev/manta-sitl-virtual")
            throw std::invalid_argument("virtual control service requires --simulation-udp");
        manta::validate_control_config(config);
        if (check) {
            std::cout << "Native transport configuration valid\n";
            return 0;
        }
        signal(SIGINT, stop);
        signal(SIGTERM, stop);
        signal(SIGPIPE, SIG_IGN);
        auto commands = socket_fd(SOCK_DGRAM), telemetry = socket_fd(SOCK_DGRAM);
        Fd link;
        if (bind(commands.value, reinterpret_cast<const sockaddr *>(&command_address),
                 sizeof(command_address)) < 0)
            throw std::runtime_error("command port already in use or unavailable");
        if (simulation) {
            link = socket_fd(SOCK_DGRAM);
            if (connect(link.value, reinterpret_cast<const sockaddr *>(&peer), sizeof(peer)) < 0)
                throw std::runtime_error("simulation connect failed");
        }
        const auto mp = config.value("mission_planner_tcp", Json::object());
        Fd listener;
        std::vector<Client> clients;
        if (mp.value("enabled", false)) {
            const auto endpoint =
                address(mp.value("host", std::string("127.0.0.1")), mp.value("port", 5760));
            if (simulation && !loopback(endpoint))
                throw std::invalid_argument("simulation Mission Planner endpoint must be loopback");
            listener = socket_fd(SOCK_STREAM);
            int reuse = 1;
            setsockopt(listener.value, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse));
            if (bind(listener.value, reinterpret_cast<const sockaddr *>(&endpoint),
                     sizeof(endpoint)) < 0 ||
                listen(listener.value, 2) < 0)
                throw std::runtime_error("Mission Planner listener unavailable");
        }
        std::deque<std::vector<uint8_t>> tx;
        size_t tx_bytes = 0;
        double next_open = 0, opened_at = 0;
        bool previously_connected = false, link_fault = false;
        const auto queue = [&](std::span<const uint8_t> bytes) {
            if (link.value < 0)
                return;
            if (tx_bytes + bytes.size() > 16384) {
                link_fault = true;
                return;
            }
            tx.emplace_back(bytes.begin(), bytes.end());
            tx_bytes += bytes.size();
        };
        const auto publish = [&](const Json &event) {
            const auto public_event = manta::export_wall_clock(event, wall_ms(), system_wall_ms());
            const auto data = public_event.dump(-1, ' ', false, Json::error_handler_t::replace);
            sendto(telemetry.value, data.data(), data.size(), MSG_DONTWAIT,
                   reinterpret_cast<const sockaddr *>(&telemetry_address),
                   sizeof(telemetry_address));
            if (event.value("type", std::string()) == "log")
                std::cerr << event["payload"].dump() << '\n';
        };
        manta::MavlinkCore core(config, queue, publish, [&]() {
            tx.clear();
            tx_bytes = 0;
        });
        manta::ControlRuntime control(config, core, publish);
        const auto flush = [&]() {
            if (link.value < 0)
                return;
            for (int count = 0; count < 32 && !tx.empty(); ++count) {
                auto &front = tx.front();
                const auto sent = write(link.value, front.data(), front.size());
                if (sent < 0) {
                    if (errno != EAGAIN && errno != EWOULDBLOCK && errno != EINTR &&
                        !(simulation && errno == ECONNREFUSED))
                        link_fault = true;
                    break;
                }
                if (sent == 0)
                    break;
                tx_bytes -= sent;
                if (size_t(sent) == front.size())
                    tx.pop_front();
                else {
                    front.erase(front.begin(), front.begin() + sent);
                    break;
                }
            }
        };
        std::cout << "Native control transport ready; command port "
                  << ntohs(command_address.sin_port) << std::endl;
        while (!stopping) {
            const double clock = monotonic_ms();
            if (!simulation && link.value < 0 && clock >= next_open) {
                try {
                    link = serial(serial_path, baudrate);
                    opened_at = clock;
                    std::cerr << "Serial opened: " << serial_path << '\n';
                } catch (const std::exception &error) {
                    std::cerr << error.what() << '\n';
                }
                next_open = clock + 2000;
            }
            uint8_t buffer[8192];
            // Both byte count and elapsed time bound input work, so a busy
            // peer cannot starve lease expiry, heartbeat or control ticks.
            const double receive_started = monotonic_ms();
            for (int i = 0; link.value >= 0 && i < 32 && monotonic_ms() - receive_started < 3;
                 ++i) {
                const auto count = read(link.value, buffer, sizeof(buffer));
                if (count < 0) {
                    if (errno != EAGAIN && errno != EWOULDBLOCK && errno != EINTR &&
                        !(simulation && errno == ECONNREFUSED))
                        link_fault = true;
                    break;
                }
                if (count == 0)
                    break;
                core.receive(std::span<const uint8_t>(buffer, count), wall_ms(), monotonic_ms());
                for (auto &client : clients) {
                    if (client.pending.size() + count > 65536)
                        client.fd.close();
                    else
                        client.pending.insert(client.pending.end(), buffer, buffer + count);
                }
            }
            for (int i = 0; i < 32; ++i) {
                sockaddr_in source{};
                socklen_t source_size = sizeof(source);
                const auto count =
                    recvfrom(commands.value, buffer, sizeof(buffer), MSG_DONTWAIT | MSG_TRUNC,
                             reinterpret_cast<sockaddr *>(&source), &source_size);
                if (count < 0)
                    break;
                if (count > ssize_t(sizeof(buffer)) || !loopback(source))
                    continue;
                try {
                    const auto request = Json::parse(buffer, buffer + count);
                    const auto result = control.command(request, wall_ms(), monotonic_ms());
                    if (request.contains("requestId")) {
                        const Json response = {
                            {"type", "command_result"},
                            {"payload", {{"requestId", request["requestId"]}, {"result", result}}},
                            {"timestamp", wall_ms() / 1000}};
                        const auto data =
                            manta::export_wall_clock(response, wall_ms(), system_wall_ms()).dump();
                        sendto(commands.value, data.data(), data.size(), MSG_DONTWAIT,
                               reinterpret_cast<const sockaddr *>(&source), source_size);
                    }
                } catch (const std::exception &error) {
                    std::cerr << "Rejected command packet: " << error.what() << '\n';
                }
            }
            if (listener.value >= 0) {
                sockaddr_in source{};
                socklen_t size = sizeof(source);
                Fd accepted(accept4(listener.value, reinterpret_cast<sockaddr *>(&source), &size,
                                    SOCK_NONBLOCK | SOCK_CLOEXEC));
                if (accepted.value >= 0) {
                    char host[INET_ADDRSTRLEN]{};
                    inet_ntop(AF_INET, &source.sin_addr, host, sizeof(host));
                    const auto allowed = mp.value("allowed_clients", Json::array());
                    if (clients.size() < 2 &&
                        (allowed.empty() || std::find(allowed.begin(), allowed.end(),
                                                      std::string(host)) != allowed.end()))
                        clients.push_back({std::move(accepted), {}});
                }
            }
            for (auto &client : clients) {
                if (client.fd.value < 0)
                    continue;
                const auto count = read(client.fd.value, buffer, sizeof(buffer));
                if (count == 0)
                    client.fd.close();
                else if (count > 0)
                    queue(std::span<const uint8_t>(buffer, count));
                else if (errno != EAGAIN && errno != EWOULDBLOCK && errno != EINTR)
                    client.fd.close();
                if (client.fd.value >= 0 && !client.pending.empty()) {
                    const auto sent = send(client.fd.value, client.pending.data(),
                                           client.pending.size(), MSG_DONTWAIT | MSG_NOSIGNAL);
                    if (sent > 0)
                        client.pending.erase(client.pending.begin(), client.pending.begin() + sent);
                    else if (sent < 0 && errno != EAGAIN && errno != EWOULDBLOCK && errno != EINTR)
                        client.fd.close();
                }
            }
            std::erase_if(clients, [](const Client &client) { return client.fd.value < 0; });
            core.tick(wall_ms(), monotonic_ms());
            control.tick(wall_ms(), monotonic_ms());
            flush();
            if (!simulation && link.value >= 0 &&
                (link_fault || (previously_connected && !core.connected()) ||
                 (!core.connected() && clock - opened_at > 8000))) {
                core.disconnect(wall_ms(), monotonic_ms());
                flush();
                link.close();
                tx.clear();
                tx_bytes = 0;
                link_fault = false;
                next_open = clock + 2000;
                clients.clear();
            }
            previously_connected = core.connected();
            pollfd fds[2] = {{commands.value, POLLIN, 0}, {link.value, POLLIN, 0}};
            poll(fds, 2, 5);
        }
        // Termination is a neutral stop, never an implicit arm or forced disarm.
        tx.clear();
        tx_bytes = 0;
        control.shutdown(wall_ms(), monotonic_ms());
        core.disconnect(wall_ms(), monotonic_ms());
        flush();
        return 0;
    } catch (const std::exception &error) {
        std::cerr << "manta-control: " << error.what() << '\n';
        return 1;
    }
}
