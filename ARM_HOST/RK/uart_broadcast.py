# -*- coding: utf-8 -*-
"""在所有空闲串口上同时持续广播可读文本，供用户在 PC 侧识别。
ttyS4 由主程序占着（在发 000000 0），不参与广播。
"""
import paramiko, sys

HOST, USER, PWD = "192.168.137.100", "linaro", "linaro"
PORTS = ["ttyS0", "ttyS3", "ttyS7", "ttyS8"]

SH = r"""#!/bin/sh
for p in %s; do
  stty -F /dev/$p 115200 cs8 -cstopb -parenb -crtscts clocal raw 2>/dev/null
done
bcast() {
  p=$1
  i=0
  while [ $i -lt %d ]; do
    printf '>>>>> PIN-TEST %%s NO-%%03d <<<<<\r\n' "$p" "$i" > /dev/$p
    i=$((i+1))
    sleep 0.6
  done
}
%s
wait
"""


def main():
    secs = int(sys.argv[1]) if len(sys.argv) > 1 else 600
    rounds = int(secs / 0.6)
    lines = "\n".join("bcast %s &" % p for p in PORTS)
    body = SH % (" ".join(PORTS), rounds, lines)

    ssh = paramiko.SSHClient()
    ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())
    ssh.connect(HOST, username=USER, password=PWD, timeout=20)

    def run(c, t=40):
        _, o, e = ssh.exec_command(c, timeout=t)
        return o.read().decode('utf-8', 'replace').strip(), e.read().decode('utf-8', 'replace').strip()

    sftp = ssh.open_sftp()
    with sftp.open("/tmp/pb2.sh", "w") as f:
        f.write(body)
    sftp.close()

    # 不要 pkill -f pb2.sh（会匹配到自己的 bash 命令行导致自杀）
    run("pkill -9 -f 'sh /tmp/pb2' 2>/dev/null; sleep 0.5")
    run("chmod +x /tmp/pb2.sh")
    run("nohup /bin/sh /tmp/pb2.sh >/tmp/pb2.log 2>&1 & sleep 2; echo started")
    print("广播已启动，端口:", ", ".join(PORTS), "持续 %ds" % secs)
    print("主程序 ttyS4 同时在发: 000000 0")

    # 自检：确认数据真的在出（读自己的 TX 需外部回环，这里只确认进程活着）
    o, _ = run("ps aux | grep '[p]b2.sh' | head -3")
    print("进程:", o or "(未存活!)")

    # 顺便做 ttyS4 的存在性确认
    o, _ = run("echo linaro | sudo -S lsof /dev/ttyS4 2>/dev/null | tail -2")
    print("ttyS4 占用:", o or "(无人占用)")
    ssh.close()


if __name__ == "__main__":
    main()
