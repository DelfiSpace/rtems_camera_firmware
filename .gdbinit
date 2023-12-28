# gets the current wsl interface ip 
#ip route show | grep -i default | awk '{ print $3}'

#source /home/v/RTEMS/app/hello/PyCortexMDebug/scripts/gdb.py
source gdb.py
svd_load STM32L4R9.svd 

python
import subprocess

# set exectable file to debug
elf2debug = "/home/v/RTEMS/app/rtems_camera_firmware/build/arm-rtems6-stm32l4/camera_firmware.elf"


def get_remote_ip():
    try:
        output = subprocess.check_output(
            "ip route show | grep -i default | awk '{ print $3 }'", 
            shell=True,
            text=True
        )
        remote_ip = output.strip()
        return remote_ip
    except subprocess.CalledProcessError:
        return None

# Example usage
py_remote_ip = get_remote_ip()

if py_remote_ip:
    print("Remote IP: " + py_remote_ip) 
    gdb.execute("file " + str(elf2debug))
    gdb.execute("target remote " + str(py_remote_ip) + ":2331")
    gdb.execute("load " + str(elf2debug))
else:
    print("Failed to obtain remote IP")

end
