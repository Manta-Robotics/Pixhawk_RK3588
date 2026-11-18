"""Shared configuration for recording, annotation, and log inspection."""
import json
import os
from pathlib import Path


LOGGER_DIRECTORY = Path(__file__).resolve().parent


def load_logger_config(config_path=None):
    if config_path is None:
        config_path = LOGGER_DIRECTORY / Path(os.environ.get('MANTA_LOGGER_CONFIG', 'config.json')).expanduser()
    config = json.loads(Path(config_path).expanduser().read_text(encoding='utf-8'))
    config['directory'] = str(LOGGER_DIRECTORY / Path(config['directory']).expanduser())
    default_socket = Path(config['directory']).parent / 'run' / 'experiment.sock'
    socket_name = os.environ.get('MANTA_JOURNAL_SOCKET') or config.get('journal_socket') or default_socket
    config['journal_socket'] = str(LOGGER_DIRECTORY / Path(socket_name).expanduser())
    return config
