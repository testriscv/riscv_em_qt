#include <stdio.h>

#include <csr.h>

#include <riscv_helper.h>

rv_uint_xlen *csr_get_reg_reference(csr_reg_td *csr_regs, uint16_t address)
{
    return &csr_regs[address].value;
}

/* this is only used for internal emulator use to be able to override any value regardless of access flags and write mask */
void csr_read_reg_internal(csr_reg_td *csr_regs, uint16_t address, rv_uint_xlen *out_val)
{
    *out_val = csr_regs[address].value;
}

/* this is only used for internal emulator use to be able to override any value regardless of access flags and write mask */
void csr_write_reg_internal(csr_reg_td *csr_regs, uint16_t address, rv_uint_xlen val)
{
    csr_regs[address].value = val;
}

int csr_read_reg(csr_reg_td *csr_regs, privilege_level curr_priv_mode, uint16_t address, rv_uint_xlen *out_val)
{
    if(address>CSR_ADDR_MAX)
        return RV_ACCESS_ERR;

    if(CSR_ACCESS_READ_GRANTED(curr_priv_mode, csr_regs[address].access_flags))
    {
        if(csr_regs[address].read_cb)
            return csr_regs[address].read_cb(csr_regs[address].priv, curr_priv_mode, csr_regs[address].internal_reg, out_val);

        *out_val = csr_regs[address].value;
        return RV_ACCESS_OK;
    }

    return RV_ACCESS_ERR;
}

int csr_write_reg(csr_reg_td *csr_regs, privilege_level curr_priv_mode, uint16_t address, rv_uint_xlen val)
{
    // printf("csr write %x\n", address);
    if(address>CSR_ADDR_MAX)
        return RV_ACCESS_ERR;

    if(CSR_ACCESS_WRITE_GRANTED(curr_priv_mode, csr_regs[address].access_flags))
    {
        if(csr_regs[address].write_cb)
        {
            // printf("write cb! %x "PRINTF_FMT"\n", address, val);
            return csr_regs[address].write_cb(csr_regs[address].priv, curr_priv_mode, csr_regs[address].internal_reg, val, csr_regs[address].write_mask);
        }
        
        // printf("csr addr: %x %x\n", address, csr_regs[address].value);
        csr_regs[address].value = val & csr_regs[address].write_mask;
        return RV_ACCESS_OK;
    }

    return RV_ACCESS_ERR;
}