// SPDX-License-Identifier: MIT
/*
 * setup-bootchoice - configure software boot order on omap4 devices
 */
#include <sys/mman.h>
#include <unistd.h>
#include <fcntl.h>
#include <stdint.h>
#include <sys/types.h>
#include <stdio.h>

#define WRITE_32(offs, data) *((uint32_t *)(mem + offs)) = data;
int main(int argc, char **argv)
{
   sync();
   int fd = open("/dev/mem", O_RDWR);
   if (fd < 0) {
     fprintf(stderr, "cannot access memory\n");
     return 1;
   }
   /* SYSCTRL_GENERAL_CORE */
   void *mem = mmap(NULL, 0x1000, PROT_READ, MAP_SHARED, fd, 0x4A002000);
   if (!mem) {
     return 1;
   } 
   /* ... .CONTROL_STATUS */
   uint32_t control_status = *((uint32_t *) (mem + 0x2C4));
   munmap(mem, 0x1000);

   printf("boot config pins: 0x%02x\n", control_status & 0xff);
   printf("boot device selection (BOOT_CFG[0:5]) : 0x%02x\n", control_status & 0x3f);

   mem = mmap(NULL, 0x1000, PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0x4A326000);
   if (!mem) {
     return 1;
   } 
   /* starting writing at PUBLIC_SAR_RAM_1_FREE, this may conflict with some
      power saving code 
    */
   WRITE_32(0xA0C, 0xCF00AA01); /* marker */
   WRITE_32(0xA10, 0xC); /* length */
   uint16_t *bootcfgbuf = (uint16_t *)(mem + 0xA14);
   *bootcfgbuf=0x0;  /* flags = 0 */
   bootcfgbuf++;
   *bootcfgbuf=0x43; /* 1. boot UART */
   bootcfgbuf++;
   *bootcfgbuf=0x45; /* 2. boot USB UTMI */
   bootcfgbuf++;
   *bootcfgbuf=0x05; /* 3. boot MMC1  (MMC2 = eMMC)*/
   bootcfgbuf++;
   *bootcfgbuf=0x0;  /* no fourth device */
   bootcfgbuf++;
   *bootcfgbuf=0x0;
   bootcfgbuf++;
   /* directing PUBLIC_SW_BOOT_CFG_ADDR to our buffer */
   WRITE_32(0xA00, 0x4A326A0C);
   munmap(mem, 0x1000);
   close(fd);
   fd = open("/proc/sysrq-trigger", O_WRONLY);
   if (fd < 0) {
     fprintf(stderr, "cannot reboot via sysrq\n no emergency ro mount possible, giving up\n");
     return 1;
   }
   sync();
   write(fd, "u", 1);
   sleep(1); 
   write(fd, "b", 1);
   return 0;
   close(fd); 
}
