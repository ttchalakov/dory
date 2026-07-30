#ifndef __MEM_H__
#define __MEM_H__

#include<stddef.h>

void  mem_init();

/* Boot-step inspection hook for mem_init(), weak and empty by default -- see
 * mem.c. Called once per step with that step's return code (0 = ok) before
 * mem_init() gives up, so an application on a target without a usable printf can
 * tell "cannot open flash" from "cannot mount filesystem" from "cannot open ram".
 * Steps, in order: "flash_open", "fs_mount", "ram_open". */
void  dory_mem_init_step(const char *step, int err);
struct pi_device *get_ram_ptr();
void *ram_malloc(size_t size);
void  ram_free(void *ptr, size_t size);
void  ram_read(void *dest, void *src, size_t size);
void  ram_write(void *dest, void *src, size_t size);
void *cl_ram_malloc(size_t size);
void  cl_ram_free(void *ptr, size_t size);
void  cl_ram_read(void *dest, void *src, size_t size);
void  cl_ram_write(void *dest, void *src, size_t size);
size_t load_file_to_ram(const void *dest, const char *filename);

#endif  // __MEM_H__
