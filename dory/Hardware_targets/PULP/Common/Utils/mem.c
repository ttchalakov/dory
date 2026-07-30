#include "mem.h"
#include "pmsis.h"
#include "bsp/bsp.h"
#include "bsp/fs.h"
#include "bsp/fs/readfs.h"
#include "bsp/flash.h"
#include "bsp/ram.h"

#ifdef USE_HYPERFLASH
#include "bsp/flash/hyperflash.h"
typedef struct pi_hyperflash_conf flash_conf_t;
#define flash_conf_init(conf) pi_hyperflash_conf_init(conf)
#elif defined USE_SPIFLASH
#include "bsp/flash/spiflash.h"
typedef struct pi_spiflash_conf flash_conf_t;
#define flash_conf_init(conf) pi_spiflash_conf_init(conf)
#elif defined USE_MRAM
typedef struct pi_mram_conf flash_conf_t;
#define flash_conf_init(conf) pi_mram_conf_init(conf)
#else
typedef struct pi_default_flash_conf flash_conf_t;
#define flash_conf_init(conf) pi_default_flash_conf_init(conf)
#endif

#ifdef USE_HYPERRAM
#include "bsp/ram/hyperram.h"
typedef struct pi_hyperram_conf ram_conf_t;
#define ram_conf_init(conf) pi_hyperram_conf_init(conf)
#else
typedef struct pi_default_ram_conf ram_conf_t;
#define ram_conf_init(conf) pi_default_ram_conf_init(conf)
#endif

#define BUFFER_SIZE 128
static uint8_t buffer[BUFFER_SIZE];

static struct pi_device flash;
static flash_conf_t flash_conf;

static struct pi_device fs;
static struct pi_readfs_conf fs_conf;

static struct pi_device ram;
static ram_conf_t ram_conf;


/* Weak, empty by default. Every failure below reports itself with printf() and
 * then calls pmsis_exit(), which on a target whose console is not a UART -- the
 * Crazyflie AI-deck, where printf goes to pins nothing is listening to -- means
 * the application simply stops mid-boot with no output at all. That is
 * indistinguishable from a wedged chip, a bad flash and an empty readfs, and each
 * of those has a completely different fix.
 *
 * An application that defines dory_mem_init_step() sees each step and its return
 * code as it happens, over whatever transport it actually has. `err` is 0 on
 * success, so a healthy boot emits one line per step and the last step reached
 * localises a failure. The hook is called BEFORE pmsis_exit(), which is the only
 * moment the information still exists. */
__attribute__((weak)) void dory_mem_init_step(const char *step, int err) {
  (void) step; (void) err;
}

void mem_init() {
  int err;

  flash_conf_init(&flash_conf);
  pi_open_from_conf(&flash, &flash_conf);
  err = pi_flash_open(&flash);
  dory_mem_init_step("flash_open", err);
  if (err) {
    printf("ERROR: Cannot open flash! Exiting...\n");
    pmsis_exit(-1);
  }

  /* Open filesystem on flash. */
  pi_readfs_conf_init(&fs_conf);
  fs_conf.fs.flash = &flash;
  pi_open_from_conf(&fs, &fs_conf);
  err = pi_fs_mount(&fs);
  dory_mem_init_step("fs_mount", err);
  if (err) {
    printf("ERROR: Cannot mount filesystem! Exiting...\n");
    pmsis_exit(-2);
  }

  ram_conf_init(&ram_conf);
  pi_open_from_conf(&ram, &ram_conf);
  err = pi_ram_open(&ram);
  dory_mem_init_step("ram_open", err);
  if (err) {
    printf("ERROR: Cannot open ram! Exiting...\n");
    pmsis_exit(-3);
  }
}

struct pi_device *get_ram_ptr() {
 return &ram;
}

void *ram_malloc(size_t size) {
  void *ptr = NULL;
  pi_ram_alloc(&ram, &ptr, size);
  return ptr;
}

void ram_free(void *ptr, size_t size) {
  pi_ram_free(&ram, ptr, size);
}

void ram_read(void *dest, void *src, const size_t size) {
  pi_ram_read(&ram, src, dest, size);
}

void ram_write(void *dest, void *src, const size_t size) {
  pi_ram_write(&ram, dest, src, size);
}

void *cl_ram_malloc(size_t size) {
  int addr;
  pi_cl_ram_req_t req;
  pi_cl_ram_alloc(&ram, size, &req);
  pi_cl_ram_alloc_wait(&req, &addr);
  return (void *) addr;
}

void cl_ram_free(void *ptr, size_t size) {
  pi_cl_ram_req_t req;
  pi_cl_ram_free(&ram, ptr, size, &req);
  pi_cl_ram_free_wait(&req);
}

void cl_ram_read(void *dest, void *src, const size_t size) {
  pi_cl_ram_req_t req;
  pi_cl_ram_read(&ram, src, dest, size, &req);
  pi_cl_ram_read_wait(&req);
}

void cl_ram_write(void *dest, void *src, const size_t size) {
  pi_cl_ram_req_t req;
  pi_cl_ram_write(&ram, dest, src, size, &req);
  pi_cl_ram_write_wait(&req);
}

size_t load_file_to_ram(const void *dest, const char *filename) {
  pi_fs_file_t *fd = pi_fs_open(&fs, filename, 0);
  if (fd == NULL) {
    printf("ERROR: Cannot open file %s! Exiting...", filename);
    pmsis_exit(-4);
  }

  const size_t size = fd->size;

  size_t offset = 0;
  do {
    const size_t read_bytes = pi_fs_read(fd, buffer, BUFFER_SIZE);
    ram_write(dest + offset, buffer, read_bytes);
    offset += read_bytes;
  } while (offset < size);

  return offset;
}
