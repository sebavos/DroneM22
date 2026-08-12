import paramiko

ssh = paramiko.SSHClient()
ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())
ssh.connect("192.168.14.8", username="uavlab1", password="UAVLAB1")

print("Ejecutando prueba de puertos TCP post-registro...")
stdin, stdout, stderr = ssh.exec_command("python3 /home/uavlab1/test_tcp_after_register.py")
print(stdout.read().decode())
print(stderr.read().decode())
ssh.close()
