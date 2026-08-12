import paramiko

ssh = paramiko.SSHClient()
ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())
ssh.connect("192.168.14.8", username="uavlab1", password="UAVLAB1")

# Upload the file
sftp = ssh.open_sftp()
sftp.put("check_nal.py", "/home/uavlab1/check_nal.py")
sftp.close()

# Run it
stdin, stdout, stderr = ssh.exec_command("python3 /home/uavlab1/check_nal.py")
print(stdout.read().decode())
ssh.close()
