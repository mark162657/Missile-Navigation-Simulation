from dataclasses import dataclass
from cryptography.hazmat.backends import default_backend
from cryptography.hazmat.primitives.ciphers import Cipher, algorithms, modes
from os import urandom
import base64

@dataclass
class TerminalParam:
    launch_platform_name: str
    launch_platform_id: int
    terminal_id: int
    launch_pass: str

class LaunchTerminal:
    def encrypt_data(key: bytes, plaintext: str) -> str:
        # Generate a random initialization vector
        iv = urandom(16)

        # Create a Cipher object
        cipher = Cipher(algorithms.AES(key), modes.CFB(iv), backend=default_backend())
        encryptor = cipher.encryptor()

        # Encrypt the plaintext
        ciphertext = encryptor.update(plaintext.encode()) + encryptor.finalize()

        # Return the IV and ciphertext as a base64 encoded string
        return base64.b64encode(iv + ciphertext).decode('utf-8')