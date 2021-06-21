# Procedure to setup ubuntu on HP Elitebook 840 (aka Micron laptop)
1. BIOS setup (F9) setting  
    - Untick Secureboot  
    - Boot option: UEFI Native (without CSM)  
2. Install Ubuntu:  
    -BIOS option - Run from External USB Drive   
3. After finishing installation of ubuntu, go to BIOS setup (F9) on restart  
4. Add custom boot option \EFI\ubuntu\grubx64.efi  
5. Change order of boot (custom boot option as first option)  
6. Restart PC  

# If still doesn't boot (Unable to find bootable systenm etc)
Try Manual boot   
7. Press Esc while restart  
8. Select boot options  
9. Select EFI  
10. Select Ubuntu  
11. Select grubx64.efi  

If boot: Restart on ubuntu and redo step 4-6  
If still doesn't boot: Reinstall ubuntu, repeat steps from start  

# WHAT TO DO IF GRUB DOESN'T LOAD EVEN THOUGH UBUNTU HAS BEEN INSTALLED

1. Plug the Ubuntu installation thumb drive and boot the USB
2. Select the option "try ubuntu without installation"
3. Connect to the internet
4. open a terminal and type

sudo apt-add-repository ppa:yannubuntu/boot-repair
sudo apt-get update
sudo apt-get install -y boot-repair
boot-repair

5. Select the "Recommended Repair" option and let it run
6. After the program notified that the repair is complete, restart the computer and try to boot the installed ubuntu

reference:
https://www.makeuseof.com/tag/fix-ubuntu-linux-pc-wont-boot/
