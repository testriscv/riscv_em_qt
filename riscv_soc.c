#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#include <riscv_config.h>
#include <riscv_helper.h>
#include <riscv_soc.h>

#define INIT_MEM_ACCESS_STRUCT(_ref, _entry, _read_func, _write_func, _priv, _addr_start, _mem_size) \
{ \
    size_t _tmp_count = _entry; \
    _ref[_tmp_count].read = _read_func; \
    _ref[_tmp_count].write = _write_func; \
    _ref[_tmp_count].priv = _priv; \
    _ref[_tmp_count].addr_start = _addr_start; \
    _ref[_tmp_count].mem_size = _mem_size; \
}

static int read_ram(void *priv, rv_uint_xlen address_internal, rv_uint_xlen *outval)
{
    uint8_t *ram_ptr = priv;
    rv_uint_xlen *xlen_ptr = (rv_uint_xlen *)&ram_ptr[address_internal];
    *outval = *xlen_ptr;
    return RV_MEM_ACCESS_OK;
}

static int write_ram(void *priv, rv_uint_xlen address_internal, rv_uint_xlen val, uint8_t nr_bytes)
{
    uint8_t *ram_ptr = priv;
    memcpy(&ram_ptr[address_internal], &val, nr_bytes);
    return RV_MEM_ACCESS_OK;
}

static int write_uart(void *priv, rv_uint_xlen address_internal, rv_uint_xlen val, uint8_t nr_bytes)
{
    (void) priv;
    (void) address_internal;
    (void) nr_bytes;

    putchar((char) val);
    return RV_MEM_ACCESS_OK;
}

static void rv_soc_init_mem_acces_cbs(rv_soc_td *rv_soc)
{
    int count = 0;
    INIT_MEM_ACCESS_STRUCT(rv_soc->mem_access_cbs, count++, read_ram, write_ram, rv_soc->ram, RAM_BASE_ADDR, RAM_SIZE_BYTES);
    INIT_MEM_ACCESS_STRUCT(rv_soc->mem_access_cbs, count++, read_clint_reg, write_clint_reg, &rv_soc->rv_core0.clint, CLINT_BASE_ADDR, CLINT_SIZE_BYTES);
    INIT_MEM_ACCESS_STRUCT(rv_soc->mem_access_cbs, count++, NULL, write_uart, NULL, UART_TX_REG_ADDR, 1);
}

static rv_uint_xlen rv_soc_read_mem(void *priv, rv_uint_xlen address, int *err)
{
    rv_soc_td *rv_soc = priv;
    rv_uint_xlen tmp_addr = 0;
    rv_uint_xlen read_val = 0;
    size_t i = 0;

    *err = RV_CORE_E_ERR;

    for(i=0;i<(sizeof(rv_soc->mem_access_cbs)/sizeof(rv_soc->mem_access_cbs[0]));i++)
    {
        if(rv_soc->mem_access_cbs[i].read != NULL)
        {
            if(ADDR_WITHIN(address, rv_soc->mem_access_cbs[i].addr_start, rv_soc->mem_access_cbs[i].mem_size))
            {
                tmp_addr = address - rv_soc->mem_access_cbs[i].addr_start;
                rv_soc->mem_access_cbs[i].read(rv_soc->mem_access_cbs[i].priv, tmp_addr, &read_val);
                *err = RV_CORE_E_OK;
                break;
            }
        }
        else
        {
            DEBUG_PRINT("Invalid Adress, or no valid read pointer found, read not executed!: "PRINTF_FMT"\n", address);
            break;
        }
    }

    return read_val;
}

static void rv_soc_write_mem(void *priv, rv_uint_xlen address, rv_uint_xlen value, uint8_t nr_bytes)
{
    rv_soc_td *rv_soc = priv;
    rv_uint_xlen tmp_addr = 0;
    size_t i = 0;

    for(i=0;i<(sizeof(rv_soc->mem_access_cbs)/sizeof(rv_soc->mem_access_cbs[0]));i++)
    {
        if(rv_soc->mem_access_cbs[i].write != NULL)
        {
            if(ADDR_WITHIN(address, rv_soc->mem_access_cbs[i].addr_start, rv_soc->mem_access_cbs[i].mem_size))
            {
                tmp_addr = address - rv_soc->mem_access_cbs[i].addr_start;
                rv_soc->mem_access_cbs[i].write(rv_soc->mem_access_cbs[i].priv, tmp_addr, value, nr_bytes);
                break;
            }
        }
        else
        {
            DEBUG_PRINT("Invalid Adress, or no valid write pointer found, write not executed!: "PRINTF_FMT"\n", address);
            break;
        }
    }

    return;
}

void rv_soc_dump_mem(rv_soc_td *rv_soc)
{
    uint32_t i = 0;
    printf("rv RAM contents\n");
    for(i=0;i<RAM_SIZE_BYTES/(sizeof(rv_uint_xlen));i++)
    {
        printf("%x\n", rv_soc->ram[i]);
    }
}

void rv_soc_init(rv_soc_td *rv_soc, char *fw_file_name)
{
    FILE * p_fw_file = NULL;
    unsigned long lsize = 0;
    size_t result = 0;

    p_fw_file = fopen(fw_file_name, "rb");
    if(p_fw_file == NULL)
    {
        printf("Could not open fw file!\n");
        exit(-1);
    }

    fseek(p_fw_file, 0, SEEK_END);
    lsize = ftell(p_fw_file);
    rewind(p_fw_file);

    if(lsize > sizeof(rv_soc->ram))
    {
        printf("Not able to load fw file of size %lu, ram space is %lu\n", lsize, sizeof(rv_soc->ram));
        exit(-2);
    }

    memset(rv_soc, 0, sizeof(rv_soc_td));

    /* initialize one core with a csr table */
    #ifdef CSR_SUPPORT
        RV_CORE_INSTANTIATE_CSR_REGS_FOR_CORE(csr_regs_core0);
        rv_core_init(&rv_soc->rv_core0, rv_soc, rv_soc_read_mem, rv_soc_write_mem, &csr_regs_core0_desc);
    #else
        rv_core_init(&rv_soc->rv_core0, rv_soc, rv_soc_read_mem, rv_soc_write_mem, NULL);
    #endif
    

    /* set some registers initial value to match qemu's */
    rv_soc->rv_core0.x[11] = 0x00001020;

    /* initialize ram and peripheral read write access pointers */
    rv_soc_init_mem_acces_cbs(rv_soc);

    result = fread(&rv_soc->ram, sizeof(char), lsize, p_fw_file);
    if(result != lsize)
    {
        printf("Error while reading file!\n");
        exit(-3);
    }

    fclose(p_fw_file);

    // rv_soc_dump_mem(rv_soc);
    // while(1);

    DEBUG_PRINT("rv SOC initialized!\n");
}
