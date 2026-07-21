"""SSH helper: run a command on the patrol car Pi and print output."""
import sys
import paramiko

HOST = "192.168.31.75"
USER = "pi"
PASS = "yahboom"

def run(cmd: str, timeout: int = 30) -> str:
    cli = paramiko.SSHClient()
    cli.set_missing_host_key_policy(paramiko.AutoAddPolicy())
    cli.connect(HOST, username=USER, password=PASS, timeout=10,
                allow_agent=False, look_for_keys=False)
    stdin, stdout, stderr = cli.exec_command(cmd, timeout=timeout)
    out = stdout.read().decode("utf-8", errors="replace")
    err = stderr.read().decode("utf-8", errors="replace")
    cli.close()
    return out + (("\n[STDERR]\n" + err) if err.strip() else "")

if __name__ == "__main__":
    cmd = sys.argv[1] if len(sys.argv) > 1 else "hostname"
    t = int(sys.argv[2]) if len(sys.argv) > 2 else 30
    print(run(cmd, t))
