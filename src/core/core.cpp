#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#include <riscv_types.h>
#include <riscv_helper.h>
#include <riscv_instr.h>

#include <core.h>

// #define CORE_DEBUG
#ifdef CORE_DEBUG
#define CORE_DBG(...) do{ printf( __VA_ARGS__ ); } while( 0 )
#else
#define CORE_DBG(...) do{ } while ( 0 )
#endif

#define ADDR_MISALIGNED(addr) (addr & 0x3)

/*
 * Functions for internal use
 */
inline void rv_core_class::prepare_sync_trap( rv_uint_xlen cause, rv_uint_xlen tval)
{
    if(!this->sync_trap_pending)
    {
        this->sync_trap_pending = 1;
        this->sync_trap_cause = cause;
        this->sync_trap_tval = tval;
    }
}

 inline privilege_level rv_core_class::check_mprv_override( bus_access_type access_type)
{
    if(access_type == bus_instr_access)
        return this->curr_priv_mode;

    int mprv = extractxlen(*this->trap.m.regs[trap_reg_status], TRAP_XSTATUS_MPRV_BIT, 1);
    privilege_level ret_val =(privilege_level) extractxlen(*this->trap.m.regs[trap_reg_status], TRAP_XSTATUS_MPP_BIT, 2);
    return mprv ? ret_val : this->curr_priv_mode;
}

rv_ret pmp_checked_bus_access(void *priv, privilege_level priv_level, bus_access_type access_type, rv_uint_xlen addr, void *value, uint8_t len)
{
    (void) priv_level;
    rv_core_td *rv_core = (rv_core_td *)priv;

    rv_uint_xlen trap_cause = (access_type == bus_instr_access) ? trap_cause_instr_access_fault :
                                                                  (access_type == bus_read_access) ? trap_cause_load_access_fault :
                                                                                                     trap_cause_store_amo_access_fault;

    if(pmp_mem_check(&rv_core->pmp, priv_level, addr, len, access_type))
    {
        printf("PMP Violation!\n");
        rv_core->prepare_sync_trap( trap_cause, addr);
        return rv_err;
    }

    return rv_core->bus_access(rv_core->priv, priv_level, access_type, addr, value, len);
}

rv_ret mmu_checked_bus_access(void *priv, privilege_level priv_level, bus_access_type access_type, rv_uint_xlen addr, void *value, uint8_t len)
{
    (void) priv_level;
    rv_core_td *rv_core = (rv_core_td *)priv;
    privilege_level internal_priv_level =rv_core->check_mprv_override( access_type);
    rv_uint_xlen trap_cause = (access_type == bus_instr_access) ? trap_cause_instr_page_fault :
                                                                  (access_type == bus_read_access) ? trap_cause_load_page_fault :
                                                                                                     trap_cause_store_amo_page_fault;
    mmu_ret mmu_ret_val = mmu_ok;

    uint8_t mxr = CHECK_BIT(*rv_core->trap.m.regs[trap_reg_status], TRAP_XSTATUS_MXR_BIT) ? 1 : 0;
    uint8_t sum = CHECK_BIT(*rv_core->trap.m.regs[trap_reg_status], TRAP_XSTATUS_SUM_BIT) ? 1 : 0;

    rv_uint_xlen tmp = 0;
    memcpy(&tmp, value, len);
    uint64_t phys_addr = mmu_virt_to_phys(&rv_core->mmu, internal_priv_level, addr, access_type, mxr, sum, &mmu_ret_val, rv_core, tmp);

    if(mmu_ret_val != mmu_ok)
    {
        rv_core->prepare_sync_trap( trap_cause, addr);
        return rv_err;
    }

    return rv_core->mmu.bus_access(rv_core->mmu.priv, internal_priv_level, access_type, phys_addr, value, len);
}

/*
 * Implementations of the RISCV instructions
 */
 void rv_core_class::instr_NOP()
{
    CORE_DBG("%s: %x\n", __func__, this->instruction);

    return;
}

/* RISCV Instructions */
 void rv_core_class::instr_LUI()
{
    CORE_DBG("%s: %x\n", __func__, this->instruction);
    this->x[this->rd] = (this->immediate << 12);
}

 void rv_core_class::instr_AUIPC()
{
    CORE_DBG("%s: %x\n", __func__, this->instruction);
    this->x[this->rd] = (this->pc) + (this->immediate << 12);
}

 void rv_core_class::instr_JAL()
{
    CORE_DBG("%s: %x\n", __func__, this->instruction);
    this->x[this->rd] = this->pc + 4;

//    if(ADDR_MISALIGNED(this->jump_offset))
//    {
//        die_msg("Addr misaligned!\n");
//        prepare_sync_trap(rv_core, trap_cause_instr_addr_misalign, 0);
//        return;
//    }

    this->next_pc = this->pc + this->jump_offset;
}

 void rv_core_class::instr_JALR()
{
    CORE_DBG("%s: %x\n", __func__, this->instruction);
    rv_uint_xlen curr_pc = this->pc + 4;
    this->jump_offset = SIGNEX_BIT_11(this->immediate);

    this->next_pc = (this->x[this->rs1] + this->jump_offset);
    this->next_pc &= ~(1<<0);

//    if(ADDR_MISALIGNED(this->next_pc))
//    {
//        die_msg("Addr misaligned!\n");
//        prepare_sync_trap(rv_core, trap_cause_instr_addr_misalign, 0);
//        return;
//    }

    this->x[this->rd] = curr_pc;
}

 void rv_core_class::instr_BEQ()
{
    CORE_DBG("%s: %x\n", __func__, this->instruction);
    if(this->x[this->rs1] == this->x[this->rs2])
    {
//        if(ADDR_MISALIGNED(this->jump_offset))
//        {
//            die_msg("Addr misaligned!\n");
//            prepare_sync_trap(rv_core, trap_cause_instr_addr_misalign, 0);
//            return;
//        }

        this->next_pc = this->pc + this->jump_offset;
    }
}

 void rv_core_class::instr_BNE()
{
    CORE_DBG("%s: %x\n", __func__, this->instruction);
    if(this->x[this->rs1] != this->x[this->rs2])
    {
//        if(ADDR_MISALIGNED(this->jump_offset))
//        {
//            die_msg("Addr misaligned!\n");
//            prepare_sync_trap(rv_core, trap_cause_instr_addr_misalign, 0);
//            return;
//        }

        this->next_pc = this->pc + this->jump_offset;
    }
}

 void rv_core_class::instr_BLT()
{
    CORE_DBG("%s: %x\n", __func__, this->instruction);
    rv_int_xlen signed_rs = this->x[this->rs1];
    rv_int_xlen signed_rs2 = this->x[this->rs2];

    if(signed_rs < signed_rs2)
    {
//        if(ADDR_MISALIGNED(this->jump_offset))
//        {
//            die_msg("Addr misaligned!\n");
//            prepare_sync_trap(rv_core, trap_cause_instr_addr_misalign, 0);
//            return;
//        }

        this->next_pc = this->pc + this->jump_offset;
    }
}

 void rv_core_class::instr_BGE()
{
    CORE_DBG("%s: %x\n", __func__, this->instruction);
    rv_int_xlen signed_rs = this->x[this->rs1];
    rv_int_xlen signed_rs2 = this->x[this->rs2];

    if(signed_rs >= signed_rs2)
    {
//        if(ADDR_MISALIGNED(this->jump_offset))
//        {
//            die_msg("Addr misaligned!\n");
//            prepare_sync_trap(rv_core, trap_cause_instr_addr_misalign, 0);
//            return;
//        }

        this->next_pc = this->pc + this->jump_offset;
    }
}

 void rv_core_class::instr_BLTU()
{
    CORE_DBG("%s: %x\n", __func__, this->instruction);
    if(this->x[this->rs1] < this->x[this->rs2])
    {
//        if(ADDR_MISALIGNED(this->jump_offset))
//        {
//            die_msg("Addr misaligned!\n");
//            prepare_sync_trap(rv_core, trap_cause_instr_addr_misalign, 0);
//            return;
//        }

        this->next_pc = this->pc + this->jump_offset;
    }
}

 void rv_core_class::instr_BGEU()
{
    CORE_DBG("%s: %x\n", __func__, this->instruction);
    if(this->x[this->rs1] >= this->x[this->rs2])
    {
//        if(ADDR_MISALIGNED(this->jump_offset))
//        {
//            die_msg("Addr misaligned!\n");
//            prepare_sync_trap(rv_core, trap_cause_instr_addr_misalign, 0);
//            return;
//        }

        this->next_pc = this->pc + this->jump_offset;
    }
}

 void rv_core_class::instr_ADDI()
{
    CORE_DBG("%s: %x " PRINTF_FMT"\n", __func__, this->instruction, this->pc);
    rv_int_xlen signed_immediate = SIGNEX_BIT_11(this->immediate);
    rv_int_xlen signed_rs_val = this->x[this->rs1];
    CORE_DBG("%s: " PRINTF_FMT" " PRINTF_FMT" " PRINTF_FMT" %x\n", __func__, this->x[this->rs1], signed_rs_val, signed_immediate, this->rs1);
    this->x[this->rd] = (signed_immediate + signed_rs_val);
}

 void rv_core_class::instr_SLTI()
{
    CORE_DBG("%s: %x\n", __func__, this->instruction);
    rv_int_xlen signed_immediate = SIGNEX_BIT_11(this->immediate);
    rv_int_xlen signed_rs_val = this->x[this->rs1];

    if(signed_rs_val < signed_immediate)
        this->x[this->rd] = 1;
    else
        this->x[this->rd] = 0;
}

 void rv_core_class::instr_SLTIU()
{
    CORE_DBG("%s: %x\n", __func__, this->instruction);
    rv_uint_xlen unsigned_immediate = SIGNEX_BIT_11(this->immediate);
    rv_uint_xlen unsigned_rs_val = this->x[this->rs1];

    if(unsigned_rs_val < unsigned_immediate)
        this->x[this->rd] = 1;
    else
        this->x[this->rd] = 0;
}

 void rv_core_class::instr_XORI()
{
    CORE_DBG("%s: %x\n", __func__, this->instruction);
    rv_int_xlen signed_immediate = SIGNEX_BIT_11(this->immediate);
    this->immediate = signed_immediate;

    if(signed_immediate == -1)
        this->x[this->rd] = this->x[this->rs1] ^ -1;
    else
        this->x[this->rd] = this->x[this->rs1] ^ this->immediate;
}

 void rv_core_class::instr_ORI()
{
    CORE_DBG("%s: %x\n", __func__, this->instruction);
    this->immediate = SIGNEX_BIT_11(this->immediate);
    this->x[this->rd] = this->x[this->rs1] | this->immediate;
}

 void rv_core_class::instr_ANDI()
{
    CORE_DBG("%s: %x\n", __func__, this->instruction);
    this->immediate = SIGNEX_BIT_11(this->immediate);
    this->x[this->rd] = this->x[this->rs1] & this->immediate;
}

 void rv_core_class::instr_SLLI()
{
    CORE_DBG("%s: %x\n", __func__, this->instruction);
    this->x[this->rd] = (this->x[this->rs1] << (this->immediate & SHIFT_OP_MASK));
}

 void rv_core_class::instr_SRAI()
{
    CORE_DBG("%s: %x\n", __func__, this->instruction);
    rv_int_xlen rs_val = this->x[this->rs1];

    /* a right shift on signed ints seem to be always arithmetic */
    rs_val = rs_val >> (this->immediate & SHIFT_OP_MASK);
    this->x[this->rd] = rs_val;
}

 void rv_core_class::instr_SRLI()
{
    CORE_DBG("%s: %x\n", __func__, this->instruction);
    this->x[this->rd] = (this->x[this->rs1] >> (this->immediate & SHIFT_OP_MASK));
}

 void rv_core_class::instr_ADD()
{
    CORE_DBG("%s: %x\n", __func__, this->instruction);
    CORE_DBG("%s: " PRINTF_FMT " %x\n", __func__, this->x[this->rs1], this->rs1);
    this->x[this->rd] = this->x[this->rs1] + this->x[this->rs2];
}

 void rv_core_class::instr_SUB()
{
    CORE_DBG("%s: %x\n", __func__, this->instruction);
    this->x[this->rd] = this->x[this->rs1] - this->x[this->rs2];
}

 void rv_core_class::instr_SLL()
{
    CORE_DBG("%s: %x\n", __func__, this->instruction);
    this->x[this->rd] = this->x[this->rs1] << (this->x[this->rs2] & SHIFT_OP_MASK);
}

 void rv_core_class::instr_SLT()
{
    CORE_DBG("%s: %x\n", __func__, this->instruction);
    rv_int_xlen signed_rs = this->x[this->rs1];
    rv_int_xlen signed_rs2 = this->x[this->rs2];

    if(signed_rs < signed_rs2) this->x[this->rd] = 1;
    else this->x[this->rd] = 0;
}

 void rv_core_class::instr_SLTU()
{
    CORE_DBG("%s: %x\n", __func__, this->instruction);
    if(this->rs1 == 0)
    {
        if(this->x[this->rs2])
            this->x[this->rd] = 1;
        else
            this->x[this->rd] = 0;
    }
    else
    {
        if(this->x[this->rs1] < this->x[this->rs2])
            this->x[this->rd] = 1;
        else
            this->x[this->rd] = 0;
    }
}

 void rv_core_class::instr_XOR()
{
    CORE_DBG("%s: %x\n", __func__, this->instruction);
    this->x[this->rd] = this->x[this->rs1] ^ this->x[this->rs2];
}

 void rv_core_class::instr_SRL()
{
    CORE_DBG("%s: %x\n", __func__, this->instruction);
    this->x[this->rd] = this->x[this->rs1] >> (this->x[this->rs2] & SHIFT_OP_MASK);
}

 void rv_core_class::instr_OR()
{
    CORE_DBG("%s: %x\n", __func__, this->instruction);
    this->x[this->rd] = this->x[this->rs1] | (this->x[this->rs2]);
}

 void rv_core_class::instr_AND()
{
    CORE_DBG("%s: %x\n", __func__, this->instruction);
    this->x[this->rd] = this->x[this->rs1] & (this->x[this->rs2]);
}

 void rv_core_class::instr_SRA()
{
    CORE_DBG("%s: %x\n", __func__, this->instruction);
    rv_int_xlen signed_rs = this->x[this->rs1];
    this->x[this->rd] = signed_rs >> (this->x[this->rs2] & SHIFT_OP_MASK);
}

 void rv_core_class::instr_LB()
{
    CORE_DBG("%s: %x\n", __func__, this->instruction);
    uint8_t tmp_load_val = 0;
    rv_int_xlen signed_offset = SIGNEX_BIT_11(this->immediate);
    rv_uint_xlen address = this->x[this->rs1] + signed_offset;
    if(mmu_checked_bus_access(this, this->curr_priv_mode, bus_read_access, address, &tmp_load_val, 1) == rv_ok)
        this->x[this->rd] = SIGNEX_BIT_7(tmp_load_val);
}

 void rv_core_class::instr_LH()
{
    CORE_DBG("%s: %x\n", __func__, this->instruction);
    uint16_t tmp_load_val = 0;
    rv_int_xlen signed_offset = SIGNEX_BIT_11(this->immediate);
    rv_uint_xlen address = this->x[this->rs1] + signed_offset;
    if(mmu_checked_bus_access(this, this->curr_priv_mode, bus_read_access, address, &tmp_load_val, 2) == rv_ok)
        this->x[this->rd] = SIGNEX_BIT_15(tmp_load_val);
}

 void rv_core_class::instr_LW()
{
    CORE_DBG("%s: %x\n", __func__, this->instruction);
    int32_t tmp_load_val = 0;
    rv_int_xlen signed_offset = SIGNEX_BIT_11(this->immediate);
    rv_uint_xlen address = this->x[this->rs1] + signed_offset;
    if(mmu_checked_bus_access(this, this->curr_priv_mode, bus_read_access, address, &tmp_load_val, 4) == rv_ok)
        this->x[this->rd] = tmp_load_val;
}

 void rv_core_class::instr_LBU()
{
    CORE_DBG("%s: %x\n", __func__, this->instruction);
    uint8_t tmp_load_val = 0;
    rv_int_xlen signed_offset = SIGNEX_BIT_11(this->immediate);
    rv_uint_xlen address = this->x[this->rs1] + signed_offset;
    if(mmu_checked_bus_access(this, this->curr_priv_mode, bus_read_access, address, &tmp_load_val, 1) == rv_ok)
        this->x[this->rd] = tmp_load_val;
}

 void rv_core_class::instr_LHU()
{
    CORE_DBG("%s: %x\n", __func__, this->instruction);
    uint16_t tmp_load_val = 0;
    rv_int_xlen signed_offset = SIGNEX_BIT_11(this->immediate);
    rv_uint_xlen address = this->x[this->rs1] + signed_offset;
    if(mmu_checked_bus_access(this, this->curr_priv_mode, bus_read_access, address, &tmp_load_val, 2) == rv_ok)
        this->x[this->rd] = tmp_load_val;
}

 void rv_core_class::instr_SB()
{
    CORE_DBG("%s: %x\n", __func__, this->instruction);
    rv_int_xlen signed_offset = SIGNEX_BIT_11(this->immediate);
    rv_uint_xlen address = this->x[this->rs1] + signed_offset;
    uint8_t value_to_write = (uint8_t)this->x[this->rs2];
    mmu_checked_bus_access(this, this->curr_priv_mode, bus_write_access, address, &value_to_write, 1);
}

 void rv_core_class::instr_SH()
{
    CORE_DBG("%s: %x\n", __func__, this->instruction);
    rv_int_xlen signed_offset = SIGNEX_BIT_11(this->immediate);
    rv_uint_xlen address = this->x[this->rs1] + signed_offset;
    uint16_t value_to_write = (uint16_t)this->x[this->rs2];
    mmu_checked_bus_access(this, this->curr_priv_mode, bus_write_access, address, &value_to_write, 2);
}

 void rv_core_class::instr_SW()
{
    CORE_DBG("%s: %x\n", __func__, this->instruction);
    rv_int_xlen signed_offset = SIGNEX_BIT_11(this->immediate);
    rv_uint_xlen address = this->x[this->rs1] + signed_offset;
    rv_uint_xlen value_to_write = (rv_uint_xlen)this->x[this->rs2];
    mmu_checked_bus_access(this, this->curr_priv_mode, bus_write_access, address, &value_to_write, 4);
}

#ifdef RV64
 void rv_core_class::instr_LWU()
{
    CORE_DBG("%s: %x\n", __func__, this->instruction);
    uint32_t tmp_load_val = 0;
    rv_uint_xlen unsigned_offset = SIGNEX_BIT_11(this->immediate);
    rv_uint_xlen address = this->x[this->rs1] + unsigned_offset;
    if(mmu_checked_bus_access(this, this->curr_priv_mode, bus_read_access, address, &tmp_load_val, 4) == rv_ok)
        this->x[this->rd] = tmp_load_val;
}

 void rv_core_class::instr_LD()
{
    CORE_DBG("%s: %x\n", __func__, this->instruction);
    rv_uint_xlen tmp_load_val = 0;
    rv_int_xlen signed_offset = SIGNEX_BIT_11(this->immediate);
    rv_int_xlen address = this->x[this->rs1] + signed_offset;
    if(mmu_checked_bus_access(this, this->curr_priv_mode, bus_read_access, address, &tmp_load_val, 8) == rv_ok)
        this->x[this->rd] = tmp_load_val;
}

 void rv_core_class::instr_SD()
{
    CORE_DBG("%s: %x\n", __func__, this->instruction);
    rv_int_xlen signed_offset = SIGNEX_BIT_11(this->immediate);
    rv_uint_xlen address = this->x[this->rs1] + signed_offset;
    rv_uint_xlen value_to_write = (rv_uint_xlen)this->x[this->rs2];
    mmu_checked_bus_access(this, this->curr_priv_mode, bus_write_access, address, &value_to_write, 8);
}

 void rv_core_class::instr_SRAIW()
{
    CORE_DBG("%s: %x\n", __func__, this->instruction);
    int32_t signed_rs_val = this->x[this->rs1];
    this->x[this->rd] = (signed_rs_val >> (this->immediate & 0x1F));
}

 void rv_core_class::instr_ADDIW()
{
    CORE_DBG("%s: %x\n", __func__, this->instruction);
    int32_t signed_immediate = SIGNEX_BIT_11(this->immediate);
    int32_t signed_rs_val = this->x[this->rs1];
    this->x[this->rd] = (signed_rs_val + signed_immediate);
}

 void rv_core_class::instr_SLLIW()
{
    CORE_DBG("%s: %x\n", __func__, this->instruction);
    int32_t signed_tmp32 = (this->x[this->rs1] << (this->immediate & 0x1F)) & 0xFFFFFFFF;
    this->x[this->rd] = (rv_int_xlen)signed_tmp32;
}

 void rv_core_class::instr_SRLIW()
{
    CORE_DBG("%s: %x\n", __func__, this->instruction);
    uint32_t unsigned_rs_val = this->x[this->rs1];
    int32_t signed_tmp32 = (unsigned_rs_val >> (this->immediate & 0x1F));
    this->x[this->rd] = (rv_int_xlen)signed_tmp32;
}

 void rv_core_class::instr_SRLW()
{
    CORE_DBG("%s: %x\n", __func__, this->instruction);
    uint32_t rs1_val = this->x[this->rs1];
    uint32_t rs2_val = (this->x[this->rs2] & 0x1F);
    int32_t signed_tmp32 = rs1_val >> rs2_val;
    this->x[this->rd] = (rv_int_xlen)signed_tmp32;
}

 void rv_core_class::instr_SRAW()
{
    CORE_DBG("%s: %x\n", __func__, this->instruction);
    int32_t rs1_val_signed = this->x[this->rs1];
    uint32_t rs2_val = (this->x[this->rs2] & 0x1F);
    int32_t signed_tmp32 = rs1_val_signed >> rs2_val;
    this->x[this->rd] = (rv_int_xlen)signed_tmp32;
}

 void rv_core_class::instr_SLLW()
{
    CORE_DBG("%s: %x\n", __func__, this->instruction);
    uint32_t rs1_val = this->x[this->rs1];
    uint32_t rs2_val = (this->x[this->rs2] & 0x1F);
    int32_t signed_tmp32 = rs1_val << rs2_val;
    this->x[this->rd] = (rv_int_xlen)signed_tmp32;
}

 void rv_core_class::instr_ADDW()
{
    CORE_DBG("%s: %x\n", __func__, this->instruction);
    uint32_t rs1_val = this->x[this->rs1];
    uint32_t rs2_val = this->x[this->rs2];
    int32_t signed_tmp32 = rs1_val + rs2_val;
    this->x[this->rd] = (rv_int_xlen)signed_tmp32;
}

 void rv_core_class::instr_SUBW()
{
    CORE_DBG("%s: %x\n", __func__, this->instruction);
    uint32_t rs1_val = this->x[this->rs1];
    uint32_t rs2_val = this->x[this->rs2];
    int32_t signed_tmp32 = rs1_val - rs2_val;
    this->x[this->rd] = (rv_int_xlen)signed_tmp32;
}
#endif

#ifdef CSR_SUPPORT
inline void rv_core_class::CSRRWx( rv_uint_xlen new_val)
{
    CORE_DBG("%s: %x " PRINTF_FMT " priv level: %d\n", __func__, this->instruction, this->pc, this->curr_priv_mode);
    rv_uint_xlen csr_val = 0;
    uint16_t csr_addr = this->immediate;
    rv_uint_xlen csr_mask = csr_get_mask(this->csr_regs, csr_addr);
    rv_uint_xlen not_allowed_bits = 0;
    rv_uint_xlen new_csr_val = 0;

    if(this->rd != 0)
    {
        if(csr_read_reg(this->csr_regs, this->curr_priv_mode, csr_addr, &csr_val))
        {
            // die_msg("Error reading CSR %x " PRINTF_FMT "\n", csr_addr, this->pc);
            prepare_sync_trap( trap_cause_illegal_instr, 0);
            return;
        }
    }

    not_allowed_bits = csr_val & ~csr_mask;
    new_csr_val = not_allowed_bits | (new_val & csr_mask);

    if(csr_write_reg(this->csr_regs, this->curr_priv_mode, csr_addr, new_csr_val))
    {
        // die_msg("Error reading CSR %x " PRINTF_FMT "\n", csr_addr, this->pc);
        prepare_sync_trap( trap_cause_illegal_instr, 0);
        return;
    }

    this->x[this->rd] = csr_val & csr_mask;
}

inline void rv_core_class::CSRRSx( rv_uint_xlen new_val)
{
    CORE_DBG("%s: %x\n", __func__, this->instruction);
    rv_uint_xlen csr_val = 0;
    uint16_t csr_addr = this->immediate;
    rv_uint_xlen csr_mask = csr_get_mask(this->csr_regs, csr_addr);
    rv_uint_xlen new_csr_val = 0;

    if(csr_read_reg(this->csr_regs, this->curr_priv_mode, csr_addr, &csr_val))
    {
        // die_msg("Error reading CSR %x " PRINTF_FMT "\n", csr_addr, this->pc);
        prepare_sync_trap( trap_cause_illegal_instr, 0);
        return;
    }

    new_csr_val = (new_val & csr_mask);

    if(this->rs1 != 0)
    {
        if(csr_write_reg(this->csr_regs, this->curr_priv_mode, csr_addr, csr_val | new_csr_val))
        {
            // die_msg("Error reading CSR %x " PRINTF_FMT "\n", csr_addr, this->pc);
            prepare_sync_trap( trap_cause_illegal_instr, 0);
            return;
        }
    }

    this->x[this->rd] = csr_val & csr_mask;
}

inline void rv_core_class::CSRRCx( rv_uint_xlen new_val)
{
    CORE_DBG("%s: %x\n", __func__, this->instruction);
    rv_uint_xlen csr_val = 0;
    uint16_t csr_addr = this->immediate;
    rv_uint_xlen csr_mask = csr_get_mask(this->csr_regs, csr_addr);
    rv_uint_xlen new_csr_val = 0;

    if(csr_read_reg(this->csr_regs, this->curr_priv_mode, csr_addr, &csr_val))
    {
        // die_msg("Error reading CSR %x " PRINTF_FMT "\n", csr_addr, this->pc);
        prepare_sync_trap( trap_cause_illegal_instr, 0);
        return;
    }

    new_csr_val = (new_val & csr_mask);

    if(this->rs1 != 0)
    {
        if(csr_write_reg(this->csr_regs, this->curr_priv_mode, csr_addr, csr_val & ~new_csr_val))
        {
            // die_msg("Error reading CSR %x " PRINTF_FMT "\n", csr_addr, this->pc);
            prepare_sync_trap( trap_cause_illegal_instr, 0);
            return;
        }
    }
    this->x[this->rd] = csr_val & csr_mask;
}

 void rv_core_class::instr_CSRRW()
{
    CSRRWx( this->x[this->rs1]);
}

 void rv_core_class::instr_CSRRS()
{
    CSRRSx( this->x[this->rs1]);
}

 void rv_core_class::instr_CSRRC()
{
    CSRRCx( this->x[this->rs1]);
}

 void rv_core_class::instr_CSRRWI()
{
    CSRRWx( this->rs1);
}

 void rv_core_class::instr_CSRRSI()
{
    CSRRSx( this->rs1);
}

 void rv_core_class::instr_CSRRCI()
{
    CSRRCx( this->rs1);
}

 void rv_core_class::instr_ECALL()
{
    // printf("%s: %x from: %d\n", __func__, this->instruction, trap_cause_user_ecall + this->curr_priv_mode);
    prepare_sync_trap( trap_cause_user_ecall + this->curr_priv_mode, 0);
}

 void rv_core_class::instr_EBREAK()
{
    /* not implemented */

    CORE_DBG("%s: %x\n", __func__, this->instruction);
}

 void rv_core_class::instr_MRET()
{
    CORE_DBG("%s: " PRINTF_FMT "\n", __func__, *this->trap.m.regs[trap_reg_ip]);
    privilege_level restored_priv_level = trap_restore_irq_settings(&this->trap, this->curr_priv_mode);
    this->curr_priv_mode = restored_priv_level;
    this->next_pc = *this->trap.m.regs[trap_reg_epc];
}

 void rv_core_class::instr_SRET()
{
    CORE_DBG("%s: %x\n", __func__, this->instruction);
    privilege_level restored_priv_level = trap_restore_irq_settings(&this->trap, this->curr_priv_mode);
    this->curr_priv_mode = restored_priv_level;
    this->next_pc = *this->trap.s.regs[trap_reg_epc];
}

 void rv_core_class::instr_URET()
{
    CORE_DBG("%s: %x\n", __func__, this->instruction);
    printf("URET!\n");
    while(1);
    /* not implemented */

}
#endif

#ifdef ATOMIC_SUPPORT
 void rv_core_class::instr_LR_W()
{
    CORE_DBG("%s: %x\n", __func__, this->instruction);
    this->lr_address = this->x[this->rs1];
    this->lr_valid = 1;
    instr_LW();
}

 void rv_core_class::instr_SC_W()
{
    CORE_DBG("%s: %x\n", __func__, this->instruction);
    if(this->lr_valid && (this->lr_address == this->x[this->rs1]))
    {
        instr_SW();
        this->x[this->rd] = 0;
    }
    else
    {
        this->x[this->rd] = 1;
    }

    this->lr_valid = 0;
    this->lr_address = 0;
}

 void rv_core_class::instr_AMOSWAP_W()
{
    CORE_DBG("%s: %x\n", __func__, this->instruction);
    rv_uint_xlen address = this->x[this->rs1];
    uint32_t rs2_val = this->x[this->rs2];
    uint32_t result = 0;

    instr_LW();
    result = rs2_val;

    mmu_checked_bus_access(this, this->curr_priv_mode, bus_write_access, address, &result, 4);
}

 void rv_core_class::instr_AMOADD_W()
{
    CORE_DBG("%s: %x\n", __func__, this->instruction);
    rv_uint_xlen address = this->x[this->rs1];
    uint32_t rd_val = 0;
    uint32_t rs2_val = this->x[this->rs2];
    uint32_t result = 0;

    instr_LW();
    rd_val = this->x[this->rd];
    result = rd_val + rs2_val;

    mmu_checked_bus_access(this, this->curr_priv_mode, bus_write_access, address, &result, 4);
}

 void rv_core_class::instr_AMOXOR_W()
{
    CORE_DBG("%s: %x\n", __func__, this->instruction);
    rv_uint_xlen address = this->x[this->rs1];
    uint32_t rd_val = 0;
    uint32_t rs2_val = this->x[this->rs2];
    uint32_t result = 0;

    instr_LW();
    rd_val = this->x[this->rd];
    result = rd_val ^ rs2_val;

    mmu_checked_bus_access(this, this->curr_priv_mode, bus_write_access, address, &result, 4);
}

 void rv_core_class::instr_AMOAND_W()
{
    CORE_DBG("%s: %x\n", __func__, this->instruction);
    rv_uint_xlen address = this->x[this->rs1];
    uint32_t rd_val = 0;
    uint32_t rs2_val = this->x[this->rs2];
    uint32_t result = 0;

    instr_LW();
    rd_val = this->x[this->rd];
    result = rd_val & rs2_val;

    mmu_checked_bus_access(this, this->curr_priv_mode, bus_write_access, address, &result, 4);
}

 void rv_core_class::instr_AMOOR_W()
{
    CORE_DBG("%s: %x\n", __func__, this->instruction);
    rv_uint_xlen address = this->x[this->rs1];
    uint32_t rd_val = 0;
    uint32_t rs2_val = this->x[this->rs2];
    uint32_t result = 0;

    instr_LW();
    rd_val = this->x[this->rd];
    result = rd_val | rs2_val;

    mmu_checked_bus_access(this, this->curr_priv_mode, bus_write_access, address, &result, 4);
}

 void rv_core_class::instr_AMOMIN_W()
{
    CORE_DBG("%s: %x\n", __func__, this->instruction);
    rv_uint_xlen address = this->x[this->rs1];
    int32_t rd_val = 0;
    int32_t rs2_val = this->x[this->rs2];
    rv_uint_xlen result = 0;

    instr_LW();

    rd_val = this->x[this->rd];
    result = ASSIGN_MIN(rd_val, rs2_val);

    mmu_checked_bus_access(this, this->curr_priv_mode, bus_write_access, address, &result, 4);
}

 void rv_core_class::instr_AMOMAX_W()
{
    CORE_DBG("%s: %x\n", __func__, this->instruction);
    rv_uint_xlen address = this->x[this->rs1];
    int32_t rd_val = 0;
    int32_t rs2_val = this->x[this->rs2];
    rv_uint_xlen result = 0;

    instr_LW();

    rd_val = this->x[this->rd];
    result = ASSIGN_MAX(rd_val, rs2_val);

    mmu_checked_bus_access(this, this->curr_priv_mode, bus_write_access, address, &result, 4);
}

 void rv_core_class::instr_AMOMINU_W()
{
    CORE_DBG("%s: %x\n", __func__, this->instruction);
    rv_uint_xlen address = this->x[this->rs1];
    uint32_t rd_val = 0;
    uint32_t rs2_val = this->x[this->rs2];
    rv_uint_xlen result = 0;

    instr_LW();

    rd_val = this->x[this->rd];
    result = ASSIGN_MIN(rd_val, rs2_val);

    mmu_checked_bus_access(this, this->curr_priv_mode, bus_write_access, address, &result, 4);
}

 void rv_core_class::instr_AMOMAXU_W()
{
    CORE_DBG("%s: %x\n", __func__, this->instruction);
    rv_uint_xlen address = this->x[this->rs1];
    uint32_t rd_val = 0;
    uint32_t rs2_val = this->x[this->rs2];
    rv_uint_xlen result = 0;

    instr_LW();

    rd_val = this->x[this->rd];
    result = ASSIGN_MAX(rd_val, rs2_val);

    mmu_checked_bus_access(this, this->curr_priv_mode, bus_write_access, address, &result, 4);
}

#ifdef RV64
 void rv_core_class::instr_LR_D()
{
    CORE_DBG("%s: %x\n", __func__, this->instruction);
    this->lr_valid = 1;
    this->lr_address = this->x[this->rs1];
    instr_LD(rv_core);
}

 void rv_core_class::instr_SC_D()
{
    CORE_DBG("%s: %x\n", __func__, this->instruction);
    if(this->lr_valid && (this->lr_address == this->x[this->rs1]))
    {
        instr_SD(rv_core);
        this->x[this->rd] = 0;
    }
    else
    {
        this->x[this->rd] = 1;
    }

    this->lr_valid = 0;
    this->lr_address = 0;
}

 void rv_core_class::instr_AMOSWAP_D()
{
    CORE_DBG("%s: %x\n", __func__, this->instruction);
    rv_uint_xlen address = this->x[this->rs1];
    rv_uint_xlen rs2_val = this->x[this->rs2];
    rv_uint_xlen result = 0;

    instr_LD(rv_core);
    result = rs2_val;

    mmu_checked_bus_access(this, this->curr_priv_mode, bus_write_access, address, &result, 8);
}

 void rv_core_class::instr_AMOADD_D()
{
    CORE_DBG("%s: %x\n", __func__, this->instruction);
    rv_uint_xlen address = this->x[this->rs1];
    rv_uint_xlen rd_val = 0;
    rv_uint_xlen rs2_val = this->x[this->rs2];
    rv_uint_xlen result = 0;

    instr_LD(rv_core);
    rd_val = this->x[this->rd];
    result = rd_val + rs2_val;

    mmu_checked_bus_access(this, this->curr_priv_mode, bus_write_access, address, &result, 8);
}

 void rv_core_class::instr_AMOXOR_D()
{
    CORE_DBG("%s: %x\n", __func__, this->instruction);
    rv_uint_xlen address = this->x[this->rs1];
    rv_uint_xlen rd_val = 0;
    rv_uint_xlen rs2_val = this->x[this->rs2];
    rv_uint_xlen result = 0;

    instr_LD(rv_core);
    rd_val = this->x[this->rd];
    result = rd_val ^ rs2_val;

    mmu_checked_bus_access(this, this->curr_priv_mode, bus_write_access, address, &result, 8);
}

 void rv_core_class::instr_AMOAND_D()
{
    CORE_DBG("%s: %x\n", __func__, this->instruction);
    rv_uint_xlen address = this->x[this->rs1];
    rv_uint_xlen rd_val = 0;
    rv_uint_xlen rs2_val = this->x[this->rs2];
    rv_uint_xlen result = 0;

    instr_LD(rv_core);
    rd_val = this->x[this->rd];
    result = rd_val & rs2_val;

    mmu_checked_bus_access(this, this->curr_priv_mode, bus_write_access, address, &result, 8);
}

 void rv_core_class::instr_AMOOR_D()
{
    CORE_DBG("%s: %x\n", __func__, this->instruction);
    rv_uint_xlen address = this->x[this->rs1];
    rv_uint_xlen rd_val = 0;
    rv_uint_xlen rs2_val = this->x[this->rs2];
    rv_uint_xlen result = 0;

    instr_LD(rv_core);
    rd_val = this->x[this->rd];
    result = rd_val | rs2_val;

    mmu_checked_bus_access(this, this->curr_priv_mode, bus_write_access, address, &result, 8);
}

 void rv_core_class::instr_AMOMIN_D()
{
    CORE_DBG("%s: %x\n", __func__, this->instruction);
    rv_uint_xlen address = this->x[this->rs1];
    rv_int_xlen rd_val = 0;
    rv_int_xlen rs2_val = this->x[this->rs2];
    rv_uint_xlen result = 0;

    instr_LD(rv_core);

    rd_val = this->x[this->rd];
    result = ASSIGN_MIN(rd_val, rs2_val);

    mmu_checked_bus_access(this, this->curr_priv_mode, bus_write_access, address, &result, 8);
}

 void rv_core_class::instr_AMOMAX_D()
{
    CORE_DBG("%s: %x\n", __func__, this->instruction);
    rv_uint_xlen address = this->x[this->rs1];
    rv_int_xlen rd_val = 0;
    rv_int_xlen rs2_val = this->x[this->rs2];
    rv_uint_xlen result = 0;

    instr_LD(rv_core);

    rd_val = this->x[this->rd];
    result = ASSIGN_MAX(rd_val, rs2_val);

    mmu_checked_bus_access(this, this->curr_priv_mode, bus_write_access, address, &result, 8);
}

 void rv_core_class::instr_AMOMINU_D()
{
    CORE_DBG("%s: %x\n", __func__, this->instruction);
    rv_uint_xlen address = this->x[this->rs1];
    rv_uint_xlen rd_val = 0;
    rv_uint_xlen rs2_val = this->x[this->rs2];
    rv_uint_xlen result = 0;

    instr_LD(rv_core);

    rd_val = this->x[this->rd];
    result = ASSIGN_MIN(rd_val, rs2_val);

    mmu_checked_bus_access(this, this->curr_priv_mode, bus_write_access, address, &result, 8);
}

 void rv_core_class::instr_AMOMAXU_D()
{
    CORE_DBG("%s: %x\n", __func__, this->instruction);
    rv_uint_xlen address = this->x[this->rs1];
    rv_uint_xlen rd_val = 0;
    rv_uint_xlen rs2_val = this->x[this->rs2];
    rv_uint_xlen result = 0;

    instr_LD(rv_core);

    rd_val = this->x[this->rd];
    result = ASSIGN_MAX(rd_val, rs2_val);

    mmu_checked_bus_access(this, this->curr_priv_mode, bus_write_access, address, &result, 8);
}
#endif
#endif

#ifdef MULTIPLY_SUPPORT
 void rv_core_class::instr_DIV()
{
    CORE_DBG("%s: %x\n", __func__, this->instruction);
    rv_int_xlen signed_rs = this->x[this->rs1];
    rv_int_xlen signed_rs2 = this->x[this->rs2];

    /* division by zero */
    if(signed_rs2 == 0)
    {
        this->x[this->rd] = -1;
        return;
    }

    /* overflow */
    if(((rv_uint_xlen)signed_rs == XLEN_INT_MIN) && (signed_rs2 == -1))
    {
        this->x[this->rd] = XLEN_INT_MIN;
        return;
    }

    this->x[this->rd] = (signed_rs/signed_rs2);
}

 void rv_core_class::instr_DIVU()
{
    CORE_DBG("%s: %x\n", __func__, this->instruction);
    rv_uint_xlen unsigned_rs = this->x[this->rs1];
    rv_uint_xlen unsigned_rs2 = this->x[this->rs2];

    /* division by zero */
    if(unsigned_rs2 == 0)
    {
        this->x[this->rd] = -1;
        return;
    }

    this->x[this->rd] = (unsigned_rs/unsigned_rs2);
}

 void rv_core_class::instr_REM()
{
    CORE_DBG("%s: %x\n", __func__, this->instruction);
    rv_int_xlen signed_rs = this->x[this->rs1];
    rv_int_xlen signed_rs2 = this->x[this->rs2];

    /* division by zero */
    if(signed_rs2 == 0)
    {
        this->x[this->rd] = signed_rs;
        return;
    }

    /* overflow */
    if(((rv_uint_xlen)signed_rs == XLEN_INT_MIN) && (signed_rs2 == -1))
    {
        this->x[this->rd] = 0;
        return;
    }

    this->x[this->rd] = (signed_rs%signed_rs2);
}

 void rv_core_class::instr_REMU()
{
    CORE_DBG("%s: %x\n", __func__, this->instruction);
    rv_uint_xlen unsigned_rs = this->x[this->rs1];
    rv_uint_xlen unsigned_rs2 = this->x[this->rs2];

    /* division by zero */
    if(unsigned_rs2 == 0)
    {
        this->x[this->rd] = unsigned_rs;
        return;
    }

    this->x[this->rd] = (unsigned_rs%unsigned_rs2);
}

 void rv_core_class::instr_MUL()
{

    CORE_DBG("%s: %x\n", __func__, this->instruction);
    rv_int_xlen signed_rs = this->x[this->rs1];
    rv_int_xlen signed_rs2 = this->x[this->rs2];
    this->x[this->rd] = signed_rs * signed_rs2;
}

 void rv_core_class::instr_MULH()
{
    CORE_DBG("%s: %x\n", __func__, this->instruction);
    rv_int_xlen result_hi = 0;
    rv_int_xlen result_lo = 0;
    MUL(this->x[this->rs1], this->x[this->rs2], &result_hi, &result_lo);
    this->x[this->rd] = result_hi;
}

 void rv_core_class::instr_MULHU()
{
    CORE_DBG("%s: %x\n", __func__, this->instruction);
    rv_uint_xlen result_hi = 0;
    rv_uint_xlen result_lo = 0;
    UMUL(this->x[this->rs1], this->x[this->rs2], &result_hi, &result_lo);
    this->x[this->rd] = result_hi;
}

 void rv_core_class::instr_MULHSU()
{
    CORE_DBG("%s: %x\n", __func__, this->instruction);
    rv_int_xlen result_hi = 0;
    rv_int_xlen result_lo = 0;
    MULHSU(this->x[this->rs1], this->x[this->rs2], &result_hi, &result_lo);
    this->x[this->rd] = result_hi;
}

#ifdef RV64
 void rv_core_class::instr_MULW()
{
    CORE_DBG("%s: %x\n", __func__, this->instruction);
    int32_t signed_rs = this->x[this->rs1];
    int32_t signed_rs2 = this->x[this->rs2];
    this->x[this->rd] = (rv_int_xlen)(signed_rs * signed_rs2);
}

 void rv_core_class::instr_DIVW()
{
    CORE_DBG("%s: %x\n", __func__, this->instruction);
    int32_t signed_rs = this->x[this->rs1];
    int32_t signed_rs2 = this->x[this->rs2];
    int32_t result = 0;

    /* division by zero */
    if(signed_rs2 == 0)
    {
        this->x[this->rd] = -1;
        return;
    }

    /* overflow */
    if((signed_rs == INT32_MIN) && (signed_rs2 == -1))
    {
        this->x[this->rd] = INT32_MIN;
        return;
    }

    result = (signed_rs/signed_rs2);

    this->x[this->rd] = (rv_int_xlen)result;
}

 void rv_core_class::instr_DIVUW()
{
    CORE_DBG("%s: %x\n", __func__, this->instruction);
    uint32_t unsigned_rs = this->x[this->rs1];
    uint32_t unsigned_rs2 = this->x[this->rs2];
    uint32_t result = 0;

    /* division by zero */
    if(unsigned_rs2 == 0)
    {
        this->x[this->rd] = -1;
        return;
    }

    result = (unsigned_rs/unsigned_rs2);

    this->x[this->rd] = SIGNEX_BIT_31(result);
}

 void rv_core_class::instr_REMW()
{
    CORE_DBG("%s: %x\n", __func__, this->instruction);
    int32_t signed_rs = this->x[this->rs1];
    int32_t signed_rs2 = this->x[this->rs2];
    int32_t result = 0;

    /* division by zero */
    if(signed_rs2 == 0)
    {
        this->x[this->rd] = (rv_int_xlen)signed_rs;
        return;
    }

    /* overflow */
    if((signed_rs == INT32_MIN) && (signed_rs2 == -1))
    {
        this->x[this->rd] = 0;
        return;
    }

    result = (signed_rs%signed_rs2);

    this->x[this->rd] = (rv_int_xlen)result;
}

 void rv_core_class::instr_REMUW()
{
    CORE_DBG("%s: %x\n", __func__, this->instruction);
    uint32_t unsigned_rs = this->x[this->rs1];
    uint32_t unsigned_rs2 = this->x[this->rs2];
    uint32_t result = 0;

    /* division by zero */
    if(unsigned_rs2 == 0)
    {
        this->x[this->rd] = SIGNEX_BIT_31(unsigned_rs);
        return;
    }

    result = (unsigned_rs%unsigned_rs2);

    this->x[this->rd] = SIGNEX_BIT_31(result);
}
#endif

#endif

#ifdef ATOMIC_SUPPORT
 void rv_core_class::preparation_func5( int32_t *next_subcode)
{
    this->func5 = ((this->instruction >> 27) & 0x1F);
    *next_subcode = this->func5;
}
#endif

#ifdef RV64
 void rv_core_class::preparation_func6( int32_t *next_subcode)
{
    this->func6 = ((this->instruction >> 26) & 0x3F);
    *next_subcode = this->func6;
}
#endif

 void rv_core_class::preparation_func7( int32_t *next_subcode)
{
    this->func7 = ((this->instruction >> 25) & 0x7F);
    *next_subcode = this->func7;
}

 void rv_core_class::preparation_func7_func12_sub5_extended( int32_t *next_subcode)
{
    this->func5 = ((this->instruction >> 20) & 0x1F);
    *next_subcode = this->func5;
}

 void rv_core_class::R_type_preparation( int32_t *next_subcode)
{
    this->rd = ((this->instruction >> 7) & 0x1F);
    this->func3 = ((this->instruction >> 12) & 0x7);
    this->rs1 = ((this->instruction >> 15) & 0x1F);
    this->rs2 = ((this->instruction >> 20) & 0x1F);
    *next_subcode = this->func3;
}

 void rv_core_class::I_type_preparation( int32_t *next_subcode)
{
    this->rd = ((this->instruction >> 7) & 0x1F);
    this->func3 = ((this->instruction >> 12) & 0x7);
    this->rs1 = ((this->instruction >> 15) & 0x1F);
    this->immediate = ((this->instruction >> 20) & 0xFFF);
    *next_subcode = this->func3;
}

 void rv_core_class::S_type_preparation( int32_t *next_subcode)
{
    this->func3 = ((this->instruction >> 12) & 0x7);
    this->rs1 = ((this->instruction >> 15) & 0x1F);
    this->rs2 = ((this->instruction >> 20) & 0x1F);
    this->immediate = (((this->instruction >> 25) << 5) | ((this->instruction >> 7) & 0x1F));
    *next_subcode = this->func3;
}

 void rv_core_class::B_type_preparation( int32_t *next_subcode)
{
    this->rd = ((this->instruction >> 7) & 0x1F);
    this->func3 = ((this->instruction >> 12) & 0x7);
    this->rs1 = ((this->instruction >> 15) & 0x1F);
    this->rs2 = ((this->instruction >> 20) & 0x1F);
    this->jump_offset=((extract32(this->instruction, 8, 4) << 1) |
                          (extract32(this->instruction, 25, 6) << 5) |
                          (extract32(this->instruction, 7, 1) << 11) |
                          (extract32(this->instruction, 31, 1) << 12) );
    this->jump_offset = SIGNEX_BIT_12(this->jump_offset);
    *next_subcode = this->func3;
}

 void rv_core_class::U_type_preparation( int32_t *next_subcode)
{
    this->rd = ((this->instruction >> 7) & 0x1F);
    this->immediate = ((this->instruction >> 12) & 0xFFFFF);
    this->immediate = SIGNEX_BIT_19(this->immediate);
    *next_subcode = -1;
}

 void rv_core_class::J_type_preparation( int32_t *next_subcode)
{
    this->rd = ((this->instruction >> 7) & 0x1F);
    this->jump_offset=((extract32(this->instruction, 21, 10) << 1) |
                          (extract32(this->instruction, 20, 1) << 11) |
                          (extract32(this->instruction, 12, 8) << 12) |
                          (extract32(this->instruction, 31, 1) << 20));
    /* sign extend the 20 bit number */
    this->jump_offset = SIGNEX_BIT_20(this->jump_offset);
    *next_subcode = -1;
}

//static instruction_hook_td JALR_func3_subcode_list[] = {
//    [FUNC3_INSTR_JALR] = {nullptr, instr_JALR, nullptr},
//};
//INIT_INSTRUCTION_LIST_DESC(JALR_func3_subcode_list);

//static instruction_hook_td BEQ_BNE_BLT_BGE_BLTU_BGEU_func3_subcode_list[] = {
//    [FUNC3_INSTR_BEQ] = {nullptr, instr_BEQ, nullptr},
//    [FUNC3_INSTR_BNE] = {nullptr, instr_BNE, nullptr},
//    [FUNC3_INSTR_BLT] = {nullptr, instr_BLT, nullptr},
//    [FUNC3_INSTR_BGE] = {nullptr, instr_BGE, nullptr},
//    [FUNC3_INSTR_BLTU] = {nullptr, instr_BLTU, nullptr},
//    [FUNC3_INSTR_BGEU] = {nullptr, instr_BGEU, nullptr},
//};
//INIT_INSTRUCTION_LIST_DESC(BEQ_BNE_BLT_BGE_BLTU_BGEU_func3_subcode_list);

//static instruction_hook_td LB_LH_LW_LBU_LHU_LWU_LD_func3_subcode_list[] = {
//    [FUNC3_INSTR_LB] = {nullptr, instr_LB, nullptr},
//    [FUNC3_INSTR_LH] = {nullptr, instr_LH, nullptr},
//    [FUNC3_INSTR_LW] = {nullptr, instr_LW, nullptr},
//    [FUNC3_INSTR_LBU] = {nullptr, instr_LBU, nullptr},
//    [FUNC3_INSTR_LHU] = {nullptr, instr_LHU, nullptr},
//    #ifdef RV64
//    [FUNC3_INSTR_LWU] = {nullptr, instr_LWU, nullptr},
//    [FUNC3_INSTR_LD] = {nullptr, instr_LD, nullptr},
//    #endif
//};
//INIT_INSTRUCTION_LIST_DESC(LB_LH_LW_LBU_LHU_LWU_LD_func3_subcode_list);

//static instruction_hook_td SB_SH_SW_SD_func3_subcode_list[] = {
//    [FUNC3_INSTR_SB] = {nullptr, instr_SB, nullptr},
//    [FUNC3_INSTR_SH] = {nullptr, instr_SH, nullptr},
//    [FUNC3_INSTR_SW] = {nullptr, instr_SW, nullptr},
//    #ifdef RV64
//    [FUNC3_INSTR_SD] = {nullptr, instr_SD, nullptr},
//    #endif
//};
//INIT_INSTRUCTION_LIST_DESC(SB_SH_SW_SD_func3_subcode_list);

//#ifdef RV64
//static instruction_hook_td SRLI_SRAI_func6_subcode_list[] = {
//    [FUNC6_INSTR_SRLI] = {nullptr, instr_SRLI, nullptr},
//    [FUNC6_INSTR_SRAI] = {nullptr, instr_SRAI, nullptr},
//};
//INIT_INSTRUCTION_LIST_DESC(SRLI_SRAI_func6_subcode_list);
//#else
//static instruction_hook_td SRLI_SRAI_func7_subcode_list[] = {
//    [FUNC7_INSTR_SRLI] = {nullptr, instr_SRLI, nullptr},
//    [FUNC7_INSTR_SRAI] = {nullptr, instr_SRAI, nullptr},
//};
//INIT_INSTRUCTION_LIST_DESC(SRLI_SRAI_func7_subcode_list);
//#endif

//static instruction_hook_td ADDI_SLTI_SLTIU_XORI_ORI_ANDI_SLLI_SRLI_SRAI_func3_subcode_list[] = {
//    [FUNC3_INSTR_ADDI] = {nullptr, instr_ADDI, nullptr},
//    [FUNC3_INSTR_SLTI] = {nullptr, instr_SLTI, nullptr},
//    [FUNC3_INSTR_SLTIU] = {nullptr, instr_SLTIU, nullptr},
//    [FUNC3_INSTR_XORI] = {nullptr, instr_XORI, nullptr},
//    [FUNC3_INSTR_ORI] = {nullptr, instr_ORI, nullptr},
//    [FUNC3_INSTR_ANDI] = {nullptr, instr_ANDI, nullptr},
//    [FUNC3_INSTR_SLLI] = {nullptr, instr_SLLI, nullptr},
//    #ifdef RV64
//    [FUNC3_INSTR_SRLI_SRAI] = {preparation_func6, nullptr, &SRLI_SRAI_func6_subcode_list_desc},
//    #else
//    [FUNC3_INSTR_SRLI_SRAI] = {preparation_func7, nullptr, &SRLI_SRAI_func7_subcode_list_desc},
//    #endif
//};
//INIT_INSTRUCTION_LIST_DESC(ADDI_SLTI_SLTIU_XORI_ORI_ANDI_SLLI_SRLI_SRAI_func3_subcode_list);

//static instruction_hook_td ADD_SUB_MUL_func7_subcode_list[] = {
//    [FUNC7_INSTR_ADD] = {nullptr, instr_ADD, nullptr},
//    [FUNC7_INSTR_SUB] = {nullptr, instr_SUB, nullptr},
//    #ifdef MULTIPLY_SUPPORT
//    [FUNC7_INSTR_MUL] = {nullptr, instr_MUL, nullptr},
//    #endif
//};
//INIT_INSTRUCTION_LIST_DESC(ADD_SUB_MUL_func7_subcode_list);

//static instruction_hook_td SLL_MULH_func7_subcode_list[] = {
//    [FUNC7_INSTR_SLL] = {nullptr, instr_SLL, nullptr},
//    #ifdef MULTIPLY_SUPPORT
//    [FUNC7_INSTR_MUL] = {nullptr, instr_MULH, nullptr},
//    #endif
//};
//INIT_INSTRUCTION_LIST_DESC(SLL_MULH_func7_subcode_list);

//static instruction_hook_td SLT_MULHSU_func7_subcode_list[] = {
//    [FUNC7_INSTR_SLT] = {nullptr, instr_SLT, nullptr},
//    #ifdef MULTIPLY_SUPPORT
//    [FUNC7_INSTR_MULHSU] = {nullptr, instr_MULHSU, nullptr},
//    #endif
//};
//INIT_INSTRUCTION_LIST_DESC(SLT_MULHSU_func7_subcode_list);

//static instruction_hook_td SLTU_MULHU_func7_subcode_list[] = {
//    [FUNC7_INSTR_SLTU] = {nullptr, instr_SLTU, nullptr},
//    #ifdef MULTIPLY_SUPPORT
//    [FUNC7_INSTR_MULHU] = {nullptr, instr_MULHU, nullptr},
//    #endif
//};
//INIT_INSTRUCTION_LIST_DESC(SLTU_MULHU_func7_subcode_list);

//static instruction_hook_td XOR_DIV_func7_subcode_list[] = {
//    [FUNC7_INSTR_XOR] = {nullptr, instr_XOR, nullptr},
//    #ifdef MULTIPLY_SUPPORT
//    [FUNC7_INSTR_DIV] = {nullptr, instr_DIV, nullptr},
//    #endif
//};
//INIT_INSTRUCTION_LIST_DESC(XOR_DIV_func7_subcode_list);

//static instruction_hook_td SRL_SRA_DIVU_func7_subcode_list[] = {
//    [FUNC7_INSTR_SRL] = {nullptr, instr_SRL, nullptr},
//    [FUNC7_INSTR_SRA] = {nullptr, instr_SRA, nullptr},
//    #ifdef MULTIPLY_SUPPORT
//    [FUNC7_INSTR_DIVU] = {nullptr, instr_DIVU, nullptr},
//    #endif
//};
//INIT_INSTRUCTION_LIST_DESC(SRL_SRA_DIVU_func7_subcode_list);

//static instruction_hook_td OR_REM_func7_subcode_list[] = {
//    [FUNC7_INSTR_OR] = {nullptr, instr_OR, nullptr},
//    #ifdef MULTIPLY_SUPPORT
//    [FUNC7_INSTR_REM] = {nullptr, instr_REM, nullptr},
//    #endif
//};
//INIT_INSTRUCTION_LIST_DESC(OR_REM_func7_subcode_list);

//static instruction_hook_td AND_REMU_func7_subcode_list[] = {
//    [FUNC7_INSTR_AND] = {nullptr, instr_AND, nullptr},
//    #ifdef MULTIPLY_SUPPORT
//    [FUNC7_INSTR_REMU] = {nullptr, instr_REMU, nullptr},
//    #endif
//};
//INIT_INSTRUCTION_LIST_DESC(AND_REMU_func7_subcode_list);

//static instruction_hook_td ADD_SUB_SLL_SLT_SLTU_XOR_SRL_SRA_OR_AND_func3_subcode_list[] = {
//    [FUNC3_INSTR_ADD_SUB_MUL] = {preparation_func7, nullptr, &ADD_SUB_MUL_func7_subcode_list_desc},
//    [FUNC3_INSTR_SLL_MULH] = {preparation_func7, nullptr, &SLL_MULH_func7_subcode_list_desc},
//    [FUNC3_INSTR_SLT_MULHSU] = {preparation_func7, nullptr, &SLT_MULHSU_func7_subcode_list_desc},
//    [FUNC3_INSTR_SLTU_MULHU] = {preparation_func7, nullptr, &SLTU_MULHU_func7_subcode_list_desc},
//    [FUNC3_INSTR_XOR_DIV] = {preparation_func7, nullptr, &XOR_DIV_func7_subcode_list_desc},
//    [FUNC3_INSTR_SRL_SRA_DIVU] = {preparation_func7, nullptr, &SRL_SRA_DIVU_func7_subcode_list_desc},
//    [FUNC3_INSTR_OR_REM] = {preparation_func7, nullptr, &OR_REM_func7_subcode_list_desc},
//    [FUNC3_INSTR_AND_REMU] = {preparation_func7, nullptr, &AND_REMU_func7_subcode_list_desc},
//};
//INIT_INSTRUCTION_LIST_DESC(ADD_SUB_SLL_SLT_SLTU_XOR_SRL_SRA_OR_AND_func3_subcode_list);

//#ifdef RV64
//static instruction_hook_td SRLIW_SRAIW_func7_subcode_list[] = {
//    [FUNC7_INSTR_SRLIW] = {nullptr, instr_SRLIW, nullptr},
//    [FUNC7_INSTR_SRAIW] = {nullptr, instr_SRAIW, nullptr},
//};
//INIT_INSTRUCTION_LIST_DESC(SRLIW_SRAIW_func7_subcode_list);

//static instruction_hook_td SLLIW_SRLIW_SRAIW_ADDIW_func3_subcode_list[] = {
//    [FUNC3_INSTR_SLLIW] = {nullptr, instr_SLLIW, nullptr},
//    [FUNC3_INSTR_SRLIW_SRAIW] = {preparation_func7, nullptr, &SRLIW_SRAIW_func7_subcode_list_desc},
//    [FUNC3_INSTR_ADDIW] = {nullptr, instr_ADDIW, nullptr},
//};
//INIT_INSTRUCTION_LIST_DESC(SLLIW_SRLIW_SRAIW_ADDIW_func3_subcode_list);

//static instruction_hook_td SRLW_SRAW_DIVUW_func7_subcode_list[] = {
//    [FUNC7_INSTR_SRLW] = {nullptr, instr_SRLW, nullptr},
//    [FUNC7_INSTR_SRAW] = {nullptr, instr_SRAW, nullptr},
//    #ifdef MULTIPLY_SUPPORT
//    [FUNC7_INSTR_DIVUW] = {nullptr, instr_DIVUW, nullptr},
//    #endif
//};
//INIT_INSTRUCTION_LIST_DESC(SRLW_SRAW_DIVUW_func7_subcode_list);

//static instruction_hook_td ADDW_SUBW_MULW_func7_subcode_list[] = {
//    [FUNC7_INSTR_ADDW] = {nullptr, instr_ADDW, nullptr},
//    [FUNC7_INSTR_SUBW] = {nullptr, instr_SUBW, nullptr},
//    #ifdef MULTIPLY_SUPPORT
//    [FUNC7_INSTR_MULW] = {nullptr, instr_MULW, nullptr},
//    #endif
//};
//INIT_INSTRUCTION_LIST_DESC(ADDW_SUBW_MULW_func7_subcode_list);

//static instruction_hook_td ADDW_SUBW_SLLW_SRLW_SRAW_MULW_DIVW_DIVUW_REMW_REMUW_func3_subcode_list[] = {
//    [FUNC3_INSTR_ADDW_SUBW_MULW] = {preparation_func7, nullptr, &ADDW_SUBW_MULW_func7_subcode_list_desc},
//    [FUNC3_INSTR_SLLW] = {nullptr, instr_SLLW, nullptr},
//    [FUNC3_INSTR_SRLW_SRAW_DIVUW] = {preparation_func7, nullptr, &SRLW_SRAW_DIVUW_func7_subcode_list_desc},
//    #ifdef MULTIPLY_SUPPORT
//    [FUNC3_INSTR_DIVW] = {nullptr, instr_DIVW, nullptr},
//    [FUNC3_INSTR_REMW] = {nullptr, instr_REMW, nullptr},
//    [FUNC3_INSTR_REMUW] = {nullptr, instr_REMUW, nullptr},
//    #endif
//};
//INIT_INSTRUCTION_LIST_DESC(ADDW_SUBW_SLLW_SRLW_SRAW_MULW_DIVW_DIVUW_REMW_REMUW_func3_subcode_list);
//#endif

//#ifdef CSR_SUPPORT
//static instruction_hook_td ECALL_EBREAK_URET_func12_sub5_subcode_list[] = {
//    [FUNC5_INSTR_ECALL] = {nullptr, instr_ECALL, nullptr},
//    [FUNC5_INSTR_EBREAK] = {nullptr, instr_EBREAK, nullptr},
//    [FUNC5_INSTR_URET] = {nullptr, instr_URET, nullptr},
//};
//INIT_INSTRUCTION_LIST_DESC(ECALL_EBREAK_URET_func12_sub5_subcode_list);

//static instruction_hook_td SRET_WFI_func12_sub5_subcode_list[] = {
//    [FUNC5_INSTR_SRET] = {nullptr, instr_SRET, nullptr},
//    [FUNC5_INSTR_WFI] = {nullptr, instr_NOP, nullptr},
//};
//INIT_INSTRUCTION_LIST_DESC(SRET_WFI_func12_sub5_subcode_list);

//static instruction_hook_td ECALL_EBREAK_URET_SRET_MRET_WFI_SFENCEVMA_func7_subcode_list[] = {
//    [FUNC7_INSTR_ECALL_EBREAK_URET] = {preparation_func7_func12_sub5_extended, nullptr, &ECALL_EBREAK_URET_func12_sub5_subcode_list_desc},
//    [FUNC7_INSTR_SRET_WFI] = {preparation_func7_func12_sub5_extended, nullptr, &SRET_WFI_func12_sub5_subcode_list_desc},
//    [FUNC7_INSTR_MRET] = {nullptr, instr_MRET, nullptr},
//    [FUNC7_INSTR_SFENCEVMA] = {nullptr, instr_NOP, nullptr},
//};
//INIT_INSTRUCTION_LIST_DESC(ECALL_EBREAK_URET_SRET_MRET_WFI_SFENCEVMA_func7_subcode_list);

//static instruction_hook_td CSRRW_CSRRS_CSRRC_CSRRWI_CSRRSI_CSRRCI_ECALL_EBREAK_URET_SRET_MRET_WFI_SFENCEVMA_func3_subcode_list[] = {
//    [FUNC3_INSTR_CSRRW] = {nullptr, instr_CSRRW, nullptr},
//    [FUNC3_INSTR_CSRRS] = {nullptr, instr_CSRRS, nullptr},
//    [FUNC3_INSTR_CSRRC] = {nullptr, instr_CSRRC, nullptr},
//    [FUNC3_INSTR_CSRRWI] = {nullptr, instr_CSRRWI, nullptr},
//    [FUNC3_INSTR_CSRRSI] = {nullptr, instr_CSRRSI, nullptr},
//    [FUNC3_INSTR_CSRRCI] = {nullptr, instr_CSRRCI, nullptr},
//    [FUNC3_INSTR_ECALL_EBREAK_MRET_SRET_URET_WFI_SFENCEVMA] = {preparation_func7, nullptr, &ECALL_EBREAK_URET_SRET_MRET_WFI_SFENCEVMA_func7_subcode_list_desc}
//};
//INIT_INSTRUCTION_LIST_DESC(CSRRW_CSRRS_CSRRC_CSRRWI_CSRRSI_CSRRCI_ECALL_EBREAK_URET_SRET_MRET_WFI_SFENCEVMA_func3_subcode_list);
//#endif

//#ifdef ATOMIC_SUPPORT
//static instruction_hook_td W_LR_SC_SWAP_ADD_XOR_AND_OR_MIN_MAX_MINU_MAXU_func5_subcode_list[] = {
//    [FUNC5_INSTR_AMO_LR] = {nullptr, instr_LR_W, nullptr},
//    [FUNC5_INSTR_AMO_SC] = {nullptr, instr_SC_W, nullptr},
//    [FUNC5_INSTR_AMO_SWAP] = {nullptr, instr_AMOSWAP_W, nullptr},
//    [FUNC5_INSTR_AMO_ADD] = {nullptr, instr_AMOADD_W, nullptr},
//    [FUNC5_INSTR_AMO_XOR] = {nullptr, instr_AMOXOR_W, nullptr},
//    [FUNC5_INSTR_AMO_AND] = {nullptr, instr_AMOAND_W, nullptr},
//    [FUNC5_INSTR_AMO_OR] = {nullptr, instr_AMOOR_W, nullptr},
//    [FUNC5_INSTR_AMO_MIN] = {nullptr, instr_AMOMIN_W, nullptr},
//    [FUNC5_INSTR_AMO_MAX] = {nullptr, instr_AMOMAX_W, nullptr},
//    [FUNC5_INSTR_AMO_MINU] = {nullptr, instr_AMOMINU_W, nullptr},
//    [FUNC5_INSTR_AMO_MAXU] = {nullptr, instr_AMOMAXU_W, nullptr},
//};
//INIT_INSTRUCTION_LIST_DESC(W_LR_SC_SWAP_ADD_XOR_AND_OR_MIN_MAX_MINU_MAXU_func5_subcode_list);

//#ifdef RV64
//static instruction_hook_td D_LR_SC_SWAP_ADD_XOR_AND_OR_MIN_MAX_MINU_MAXU_func5_subcode_list[] = {
//    [FUNC5_INSTR_AMO_LR] = {nullptr, instr_LR_D, nullptr},
//    [FUNC5_INSTR_AMO_SC] = {nullptr, instr_SC_D, nullptr},
//    [FUNC5_INSTR_AMO_SWAP] = {nullptr, instr_AMOSWAP_D, nullptr},
//    [FUNC5_INSTR_AMO_ADD] = {nullptr, instr_AMOADD_D, nullptr},
//    [FUNC5_INSTR_AMO_XOR] = {nullptr, instr_AMOXOR_D, nullptr},
//    [FUNC5_INSTR_AMO_AND] = {nullptr, instr_AMOAND_D, nullptr},
//    [FUNC5_INSTR_AMO_OR] = {nullptr, instr_AMOOR_D, nullptr},
//    [FUNC5_INSTR_AMO_MIN] = {nullptr, instr_AMOMIN_D, nullptr},
//    [FUNC5_INSTR_AMO_MAX] = {nullptr, instr_AMOMAX_D, nullptr},
//    [FUNC5_INSTR_AMO_MINU] = {nullptr, instr_AMOMINU_D, nullptr},
//    [FUNC5_INSTR_AMO_MAXU] = {nullptr, instr_AMOMAXU_D, nullptr},
//};
//INIT_INSTRUCTION_LIST_DESC(D_LR_SC_SWAP_ADD_XOR_AND_OR_MIN_MAX_MINU_MAXU_func5_subcode_list);
//#endif

//static instruction_hook_td W_LR_SC_SWAP_ADD_XOR_AND_OR_MIN_MAX_MINU_MAXU_func3_subcode_list[] = {
//    [FUNC3_INSTR_W_LR_SC_SWAP_ADD_XOR_AND_OR_MIN_MAX_MINU_MAXU] = {preparation_func5, nullptr, &W_LR_SC_SWAP_ADD_XOR_AND_OR_MIN_MAX_MINU_MAXU_func5_subcode_list_desc},
//    #ifdef RV64
//    [FUNC3_INSTR_D_LR_SC_SWAP_ADD_XOR_AND_OR_MIN_MAX_MINU_MAXU] = {preparation_func5, nullptr, &D_LR_SC_SWAP_ADD_XOR_AND_OR_MIN_MAX_MINU_MAXU_func5_subcode_list_desc},
//    #endif
//};
//INIT_INSTRUCTION_LIST_DESC(W_LR_SC_SWAP_ADD_XOR_AND_OR_MIN_MAX_MINU_MAXU_func3_subcode_list);
//#endif

//static instruction_hook_td RV_opcode_list[] = {
//    [INSTR_LUI] = {U_type_preparation, instr_LUI, nullptr},
//    [INSTR_AUIPC] = {U_type_preparation, instr_AUIPC, nullptr},
//    [INSTR_JAL] = {J_type_preparation, instr_JAL, nullptr},
//    [INSTR_JALR] = {I_type_preparation, nullptr, &JALR_func3_subcode_list_desc},
//    [INSTR_BEQ_BNE_BLT_BGE_BLTU_BGEU] = {B_type_preparation, nullptr, &BEQ_BNE_BLT_BGE_BLTU_BGEU_func3_subcode_list_desc},
//    [INSTR_LB_LH_LW_LBU_LHU_LWU_LD] = {I_type_preparation, nullptr, &LB_LH_LW_LBU_LHU_LWU_LD_func3_subcode_list_desc},
//    [INSTR_SB_SH_SW_SD] = {S_type_preparation, nullptr, &SB_SH_SW_SD_func3_subcode_list_desc},
//    [INSTR_ADDI_SLTI_SLTIU_XORI_ORI_ANDI_SLLI_SRLI_SRAI] = {I_type_preparation, nullptr, &ADDI_SLTI_SLTIU_XORI_ORI_ANDI_SLLI_SRLI_SRAI_func3_subcode_list_desc},
//    [INSTR_ADD_SUB_SLL_SLT_SLTU_XOR_SRL_SRA_OR_AND_MUL_MULH_MULHSU_MULHU_DIV_DIVU_REM_REMU] = {R_type_preparation, nullptr, &ADD_SUB_SLL_SLT_SLTU_XOR_SRL_SRA_OR_AND_func3_subcode_list_desc},
//    [INSTR_FENCE_FENCE_I] = {nullptr, instr_NOP, nullptr}, /* Not implemented */

//    #ifdef RV64
//    [INSTR_ADDIW_SLLIW_SRLIW_SRAIW] = {I_type_preparation, nullptr, &SLLIW_SRLIW_SRAIW_ADDIW_func3_subcode_list_desc},
//    [INSTR_ADDW_SUBW_SLLW_SRLW_SRAW_MULW_DIVW_DIVUW_REMW_REMUW] = {R_type_preparation, nullptr, &ADDW_SUBW_SLLW_SRLW_SRAW_MULW_DIVW_DIVUW_REMW_REMUW_func3_subcode_list_desc},
//    #endif

//    #ifdef CSR_SUPPORT
//    [INSTR_ECALL_EBREAK_MRET_SRET_URET_WFI_CSRRW_CSRRS_CSRRC_CSRRWI_CSRRSI_CSRRCI_SFENCEVMA] = {I_type_preparation, nullptr, &CSRRW_CSRRS_CSRRC_CSRRWI_CSRRSI_CSRRCI_ECALL_EBREAK_URET_SRET_MRET_WFI_SFENCEVMA_func3_subcode_list_desc},
//    #endif

//    #ifdef ATOMIC_SUPPORT
//    [INSTR_AMO_W_D_LR_SC_SWAP_ADD_XOR_AND_OR_MIN_MAX_MINU_MAXU] = {R_type_preparation, nullptr, &W_LR_SC_SWAP_ADD_XOR_AND_OR_MIN_MAX_MINU_MAXU_func3_subcode_list_desc},
//    #endif
//};
//INIT_INSTRUCTION_LIST_DESC(RV_opcode_list);

//static void rv_call_from_opcode_list(, instruction_desc_td *opcode_list_desc, uint32_t opcode)
//{
//    int32_t next_subcode = -1;

//    unsigned int list_size = opcode_list_desc->instruction_hook_list_size;
//    instruction_hook_td *opcode_list = opcode_list_desc->instruction_hook_list;

//    if( (opcode_list[opcode].preparation_cb == nullptr) &&
//            (opcode_list[opcode].execution_cb == nullptr) &&
//            (opcode_list[opcode].next == nullptr) )
//        die_msg("Unknown instruction: %08x PC: " PRINTF_FMT" Cycle: %016ld\n", this->instruction, this->pc, this->curr_cycle);

//    if(opcode >= list_size)
//        die_msg("Unknown instruction: %08x PC: " PRINTF_FMT" Cycle: %016ld\n", this->instruction, this->pc, this->curr_cycle);

//    if(opcode_list[opcode].preparation_cb != nullptr)
//        opcode_list[opcode].preparation_cb(rv_core, &next_subcode);

//    if(opcode_list[opcode].execution_cb != nullptr)
//    {
//     //   this->execute_cb = opcode_list[opcode].execution_cb;
//    }
//    if((next_subcode != -1) && (opcode_list[opcode].next != nullptr))
//        rv_call_from_opcode_list(rv_core, opcode_list[opcode].next, next_subcode);
//}

#ifdef CSR_SUPPORT
void rv_core_class::rv_core_update_interrupts( uint8_t mei, uint8_t mti, uint8_t msi)
{
    trap_set_pending_bits(&this->trap, mei, mti, msi);
}

 uint8_t   rv_core_class::rv_core_prepare_interrupts()
{
    trap_cause_interrupt interrupt_cause =(trap_cause_interrupt) 0;
    trap_ret trap_retval = (trap_ret)0;
    privilege_level serving_priv_level = machine_mode;

    /* Privilege Spec: "Multiple simultaneous interrupts and traps at the same privilege level are handled in the following
         * decreasing priority order: external interrupts, software interrupts, timer interrupts, then finally any
         * synchronous traps."
         *
         * NOTE: We actually don't use this priority order here. The problem is, that if an interrupt and a synchronous
         * trap is active at the same cycle the interrupt will be handled first and in the next cycle immediately the synchronous trap
         * (as they will also occur when interrupts are globally disabled) which will potentially disrupt the privilige level at which
         * the synchronous trap should actually be handled. (This issue actually happenend when a user ecall and a timer IRQ occured at the same cycle).
         * So we use another prio order to circumvent this:
         * We handle synchronous traps at the highest prio. (Interrupts will be disabled during the handling, regardless if it is a sync trap or a IRQ)
         * This ensures atomicity among sync traps and Interrupts.
         *
         * Possible solution for this: we could remember pending sync traps for each priv level separately. And handle them only if we are in the
         * appropriate priv level. Anyway for now we just keep it simple like this, as except that we don't follow the spec here 100% it works just fine.
         * Furthermore simultanious interrupts at the same cycle should be very rare anyway.
         */


    if(this->sync_trap_pending)
    {
        serving_priv_level = (privilege_level) trap_check_exception_delegation((trap_td *)&this->trap, this->curr_priv_mode, (trap_cause_exception)this->sync_trap_cause);

        // printf("exception! serving priv: %d cause %d edeleg %x curr priv mode %x cycle %ld\n", serving_priv_level, this->sync_trap_cause, *this->trap.m.regs[trap_reg_edeleg], this->curr_priv_mode, this->curr_cycle);
        // printf("exception! serving: %x curr priv %x " PRINTF_FMT " " PRINTF_FMT " pc: " PRINTF_FMT "\n", serving_priv_level, this->curr_priv_mode, this->sync_trap_cause, *this->trap.m.regs[trap_reg_status], this->pc);
        this->pc = trap_serve_interrupt(&this->trap, serving_priv_level, this->curr_priv_mode, 0, this->sync_trap_cause, this->pc, this->sync_trap_tval);
        this->curr_priv_mode = serving_priv_level;
        this->sync_trap_pending = 0;
        this->sync_trap_cause = 0;
        this->sync_trap_tval = 0;
        return 1;
    }

    /* For simplicity we just stupidly go down from machine exti to user swi
         * Altough the correct order should be (exti, swi, timer, probably for each priv level separately)
         * Simplicity definitely wins here over spec correctness
         */
    for(int interrupt_cause_int=(int)trap_cause_machine_exti;interrupt_cause_int>=trap_cause_user_swi;interrupt_cause_int--)
    {
        interrupt_cause = (trap_cause_interrupt)interrupt_cause_int;
        trap_retval = trap_check_interrupt_pending(&this->trap, this->curr_priv_mode, interrupt_cause, &serving_priv_level);
        if(trap_retval)
        {
            this->pc = trap_serve_interrupt(&this->trap, serving_priv_level, this->curr_priv_mode, 1, interrupt_cause, this->pc, this->sync_trap_tval);
            this->curr_priv_mode = serving_priv_level;
            return 1;
        }
    }

    return 0;
}
#endif

inline rv_uint_xlen rv_core_class::rv_core_fetch()
{
    rv_uint_xlen addr = this->pc;

    return mmu_checked_bus_access(this, this->curr_priv_mode, bus_instr_access, addr, &this->instruction, sizeof(rv_uint_xlen));
}

inline rv_uint_xlen rv_core_class::rv_core_decode()
{
    this->opcode = (this->instruction & 0x7F);
    this->rd = 0;
    this->rs1 = 0;
    this->rs2 = 0;
    this->func3 = 0;
    this->func7 = 0;
    this->immediate = 0;
    this->jump_offset = 0;

    this->execute_instr = INSTR_NOP_EX ;

    /*   [INSTR_LUI] = {U_type_preparation, instr_LUI, nullptr},
    [INSTR_AUIPC] = {U_type_preparation, instr_AUIPC, nullptr},
    [INSTR_JAL] = {J_type_preparation, instr_JAL, nullptr},
    [INSTR_JALR] = {I_type_preparation, nullptr, &JALR_func3_subcode_list_desc},
    [INSTR_BEQ_BNE_BLT_BGE_BLTU_BGEU] = {B_type_preparation, nullptr, &BEQ_BNE_BLT_BGE_BLTU_BGEU_func3_subcode_list_desc},
    [INSTR_LB_LH_LW_LBU_LHU_LWU_LD] = {I_type_preparation, nullptr, &LB_LH_LW_LBU_LHU_LWU_LD_func3_subcode_list_desc},
    [INSTR_SB_SH_SW_SD] = {S_type_preparation, nullptr, &SB_SH_SW_SD_func3_subcode_list_desc},
    [INSTR_ADDI_SLTI_SLTIU_XORI_ORI_ANDI_SLLI_SRLI_SRAI] = {I_type_preparation, nullptr, &ADDI_SLTI_SLTIU_XORI_ORI_ANDI_SLLI_SRLI_SRAI_func3_subcode_list_desc},
    [INSTR_ADD_SUB_SLL_SLT_SLTU_XOR_SRL_SRA_OR_AND_MUL_MULH_MULHSU_MULHU_DIV_DIVU_REM_REMU] = {R_type_preparation, nullptr, &ADD_SUB_SLL_SLT_SLTU_XOR_SRL_SRA_OR_AND_func3_subcode_list_desc},
    [INSTR_FENCE_FENCE_I] = {nullptr, instr_NOP, nullptr}, /* Not implemented */

    /* #ifdef RV64
        [INSTR_ADDIW_SLLIW_SRLIW_SRAIW] = {I_type_preparation, nullptr, &SLLIW_SRLIW_SRAIW_ADDIW_func3_subcode_list_desc},
        [INSTR_ADDW_SUBW_SLLW_SRLW_SRAW_MULW_DIVW_DIVUW_REMW_REMUW] = {R_type_preparation, nullptr, &ADDW_SUBW_SLLW_SRLW_SRAW_MULW_DIVW_DIVUW_REMW_REMUW_func3_subcode_list_desc},
    #endif

    #ifdef CSR_SUPPORT
        [INSTR_ECALL_EBREAK_MRET_SRET_URET_WFI_CSRRW_CSRRS_CSRRC_CSRRWI_CSRRSI_CSRRCI_SFENCEVMA] = {I_type_preparation, nullptr, &CSRRW_CSRRS_CSRRC_CSRRWI_CSRRSI_CSRRCI_ECALL_EBREAK_URET_SRET_MRET_WFI_SFENCEVMA_func3_subcode_list_desc},
    #endif

    #ifdef ATOMIC_SUPPORT
        [INSTR_AMO_W_D_LR_SC_SWAP_ADD_XOR_AND_OR_MIN_MAX_MINU_MAXU] = {R_type_preparation, nullptr, &W_LR_SC_SWAP_ADD_XOR_AND_OR_MIN_MAX_MINU_MAXU_func3_subcode_list_desc},
    #endif*/


    switch ( this->opcode )
    {

    case   INSTR_LUI  :
    {

        //[INSTR_JAL] = {J_type_preparation, instr_JAL, nullptr},
        int32_t next_subcode = -1;
        U_type_preparation( &next_subcode);
        //this->execute_cb =instr_LUI;
        this->execute_instr = INSTR_LUI_EX;
        break ;
    }

    case   INSTR_AUIPC  :
    {
        //[INSTR_JAL] = {J_type_preparation, instr_JAL, nullptr},
        int32_t next_subcode = -1;
        U_type_preparation( &next_subcode);
        //this->execute_cb =instr_AUIPC;
        this->execute_instr = INSTR_AUIPC_EX;

        break ;
    }
    case   INSTR_JAL  :
    {

        //[INSTR_JAL] = {J_type_preparation, instr_JAL, nullptr},
        int32_t next_subcode = -1;
        J_type_preparation( &next_subcode);
        //this->execute_cb =instr_JAL;
        this->execute_instr = INSTR_JAL_EX;
        break ;
    }
    case   INSTR_JALR  :
    {

        int32_t next_subcode = -1;
        I_type_preparation( &next_subcode);

        switch ( next_subcode )
        {
        case   FUNC3_INSTR_JALR  :
            //this->execute_cb =instr_JALR;
            this->execute_instr = INSTR_JALR_EX;
            break;
        default:
            break;
        }

        break ;
    }

    case   INSTR_BEQ_BNE_BLT_BGE_BLTU_BGEU  :
    {

        int32_t next_subcode = -1;
        B_type_preparation( &next_subcode);

        switch ( next_subcode )
        {

        case   FUNC3_INSTR_BEQ  :
            //this->execute_cb =instr_BEQ;
            this->execute_instr = INSTR_BEQ_EX;
            break;
        case   FUNC3_INSTR_BNE  :
            //this->execute_cb =instr_BNE;
            this->execute_instr = INSTR_BNE_EX;
            break;
        case   FUNC3_INSTR_BLT  :
            //this->execute_cb =instr_BLT;
            this->execute_instr = INSTR_BLT_EX;
            break;
        case   FUNC3_INSTR_BGE  :
            //this->execute_cb =instr_BGE;
            this->execute_instr =INSTR_BGE_EX;
            break;
        case   FUNC3_INSTR_BLTU  :
            //this->execute_cb =instr_BLTU;
            this->execute_instr =INSTR_BLTU_EX ;
            break;
        case   FUNC3_INSTR_BGEU  :
            //this->execute_cb =instr_BGEU;
            this->execute_instr =INSTR_BGEU_EX ;
            break;
        default:
            break;
        }

        break ;
    }

    case   INSTR_LB_LH_LW_LBU_LHU_LWU_LD  :
    {

        int32_t next_subcode = -1;
        I_type_preparation( &next_subcode);


        switch ( next_subcode )
        {

        case   FUNC3_INSTR_LB  :
            //this->execute_cb =instr_LB;
            this->execute_instr =INSTR_LB_EX ;
            break;
        case   FUNC3_INSTR_LH  :
            //this->execute_cb =instr_LH;
            this->execute_instr =INSTR_LH_EX ;
            break;
        case   FUNC3_INSTR_LW  :
            //this->execute_cb =instr_LW;
            this->execute_instr =INSTR_LW_EX ;
            break;
        case   FUNC3_INSTR_LBU  :
            //this->execute_cb =instr_LBU;
            this->execute_instr =INSTR_LBU_EX ;
            break;
        case   FUNC3_INSTR_LHU  :
            //this->execute_cb =instr_LHU;
            this->execute_instr =INSTR_LHU_EX ;
            break;

        default:
            break;
        }

        break ;
    }
    case   INSTR_SB_SH_SW_SD  :
    {

        int32_t next_subcode = -1;
        S_type_preparation( &next_subcode);

        switch ( next_subcode )
        {

        case   FUNC3_INSTR_SB  :
            //this->execute_cb =instr_SB;
            this->execute_instr =INSTR_SB_EX;
            break;
        case   FUNC3_INSTR_SH  :
            //this->execute_cb =instr_SH;
            this->execute_instr =INSTR_SH_EX;
            break;
        case   FUNC3_INSTR_SW  :
            //this->execute_cb =instr_SW;
            this->execute_instr =INSTR_SW_EX;
            break;


        default:
            break;
        }

        break ;
    }


    case   INSTR_ADDI_SLTI_SLTIU_XORI_ORI_ANDI_SLLI_SRLI_SRAI  :
    {


        int32_t next_subcode = -1;
        I_type_preparation( &next_subcode);
        int32_t next_subcode2 = -1;
        switch ( next_subcode )
        {

        case   FUNC3_INSTR_ADDI  :
            //this->execute_cb =instr_ADDI;
            this->execute_instr =INSTR_ADDI_EX;
            break;
        case   FUNC3_INSTR_SLTI  :
            //this->execute_cb =instr_SLTI;
            this->execute_instr =INSTR_SLTI_EX;
            break;
        case   FUNC3_INSTR_SLTIU  :
            //this->execute_cb =instr_SLTIU;
            this->execute_instr =INSTR_SLTIU_EX;
            break;
        case   FUNC3_INSTR_XORI  :
            //this->execute_cb =instr_XORI;
            this->execute_instr =INSTR_XORI_EX;
            break;
        case   FUNC3_INSTR_ORI  :
            //this->execute_cb =instr_ORI;
            this->execute_instr =INSTR_ORI_EX;
            break;
        case   FUNC3_INSTR_ANDI  :
            //this->execute_cb =instr_ANDI;
            this->execute_instr =INSTR_ANDI_EX;
            break;
        case   FUNC3_INSTR_SLLI  :
            //this->execute_cb =instr_SLLI;
            this->execute_instr =INSTR_SLLI_EX;
            break;
        case   FUNC3_INSTR_SRLI_SRAI  :
        {
            preparation_func7(&next_subcode2) ;
            switch (next_subcode2 )
            {
            case FUNC7_INSTR_SRLI :
                //this->execute_cb = instr_SRLI ;
                this->execute_instr = INSTR_SRLI_F7_EX ;
                break ;
            case FUNC7_INSTR_SRAI :
                //this->execute_cb = instr_SRAI ;
                this->execute_instr = INSTR_SRAI_F7_EX ;

                break ;
            }
            break;
        }
        default:
            break;
        }

        break ;
    }

    case INSTR_ADD_SUB_SLL_SLT_SLTU_XOR_SRL_SRA_OR_AND_MUL_MULH_MULHSU_MULHU_DIV_DIVU_REM_REMU :
    {


        int32_t next_subcode = -1;
        R_type_preparation( &next_subcode);
        int32_t next_subcode2 = -1;
        switch ( next_subcode )
        {

        case   FUNC3_INSTR_ADD_SUB_MUL  :
        {
            preparation_func7(&next_subcode2) ;

            switch (next_subcode2 )
            {
            case FUNC7_INSTR_ADD :
                //this->execute_cb = instr_ADD ;
                 this->execute_instr = INSTR_ADD_EX;
                break ;
            case FUNC7_INSTR_SUB :
                //this->execute_cb = instr_SUB ;
                this->execute_instr = INSTR_SUB_EX;
                break;
#ifdef MULTIPLY_SUPPORT
            case FUNC7_INSTR_MUL :
                //this->execute_cb = instr_MUL ;
                this->execute_instr = INSTR_MUL_EX;

                break ;
#endif
            }
            break;
        }
        case   FUNC3_INSTR_SLL_MULH  :
        {
            preparation_func7(&next_subcode2) ;

            switch (next_subcode2 )
            {
            case FUNC7_INSTR_SLL :
                //this->execute_cb = instr_SLL ;
                this->execute_instr = INSTR_SLL_EX;
                break ;

#ifdef MULTIPLY_SUPPORT
            case FUNC7_INSTR_MUL :
                //this->execute_cb = instr_MULH ;
                this->execute_instr = INSTR_MUL_EX;

                break ;
#endif
            }
            break;
        }


        case   FUNC3_INSTR_SLT_MULHSU  :
        {
            preparation_func7(&next_subcode2) ;

            switch (next_subcode2 )
            {
            case FUNC7_INSTR_SLT :
                //this->execute_cb = instr_SLT ;
                this->execute_instr = INSTR_SLT_EX ;
                break ;

#ifdef MULTIPLY_SUPPORT
            case FUNC7_INSTR_MULHSU :
                //this->execute_cb = instr_MULHSU ;
                this->execute_instr = INSTR_MULHSU_EX ;
                break ;
#endif
            }
            break;
        }

        case   FUNC3_INSTR_SLTU_MULHU  :
        {
            preparation_func7(&next_subcode2) ;

            switch (next_subcode2 )
            {
            case FUNC7_INSTR_SLTU :
                //this->execute_cb = instr_SLTU ;
                this->execute_instr = INSTR_SLTU_EX ;
                break ;

#ifdef MULTIPLY_SUPPORT
            case FUNC7_INSTR_MULHU :
                //this->execute_cb = instr_MULHU ;
                this->execute_instr = INSTR_MULHU_EX ;
                break ;
#endif
            }
            break;
        }


        case   FUNC3_INSTR_XOR_DIV  :
        {
            preparation_func7(&next_subcode2) ;

            switch (next_subcode2 )
            {
            case FUNC7_INSTR_XOR :
                //this->execute_cb = instr_XOR ;
                this->execute_instr = INSTR_XOR_EX ;
                break ;

#ifdef MULTIPLY_SUPPORT
            case FUNC7_INSTR_DIV :
               // this->execute_cb = instr_DIV ;
                this->execute_instr = INSTR_DIV_EX ;
                break ;
#endif
            }
            break;
        }


        case   FUNC3_INSTR_SRL_SRA_DIVU  :
        {
            preparation_func7(&next_subcode2) ;

            switch (next_subcode2 )
            {
            case FUNC7_INSTR_SRL :
               // this->execute_cb = instr_SRL ;
                this->execute_instr = INSTR_SRL_EX ;
                break ;
            case FUNC7_INSTR_SRA :
               // this->execute_cb = instr_SRA ;
                this->execute_instr = INSTR_SRA_EX ;
                break ;

#ifdef MULTIPLY_SUPPORT
            case FUNC7_INSTR_DIVU :
               // this->execute_cb = instr_DIVU ;
                this->execute_instr = INSTR_DIVU_EX ;

                break ;
#endif
            }
            break;
        }
        case   FUNC3_INSTR_OR_REM  :
        {
            preparation_func7(&next_subcode2) ;

            switch (next_subcode2 )
            {
            case FUNC7_INSTR_OR :
               // this->execute_cb = instr_OR ;
                this->execute_instr = INSTR_OR_EX ;

                break ;


#ifdef MULTIPLY_SUPPORT
            case FUNC7_INSTR_REM :
               // this->execute_cb = instr_REM ;
                this->execute_instr = INSTR_REM_EX ;


                break ;
#endif
            }
            break;
        }

        case   FUNC3_INSTR_AND_REMU  :
        {
            preparation_func7(&next_subcode2) ;

            switch (next_subcode2 )
            {
            case FUNC7_INSTR_AND :
               //this->execute_cb = instr_AND ;
                this->execute_instr = INSTR_AND_EX ;
                break ;


#ifdef MULTIPLY_SUPPORT
            case FUNC7_INSTR_REMU :
              //  this->execute_cb = instr_REMU ;
                this->execute_instr = INSTR_REMU_EX ;
                break ;
#endif
            }
            break;
        }


        default:

            break;
        }

        break ;
    }
    case INSTR_FENCE_FENCE_I :
        //NOT implemented
       // this->execute_cb = instr_NOP ;
        this->execute_instr = INSTR_NOP_EX ;
        break ;

#ifdef CSR_SUPPORT
    case INSTR_ECALL_EBREAK_MRET_SRET_URET_WFI_CSRRW_CSRRS_CSRRC_CSRRWI_CSRRSI_CSRRCI_SFENCEVMA :
    {

        int32_t next_subcode = -1;
        I_type_preparation( &next_subcode);
        int32_t next_subcode2 = -1;
        int32_t next_subcode3 = -1;
        switch ( next_subcode )
        {

        case   FUNC3_INSTR_CSRRW  :
           // this->execute_cb =instr_CSRRW;
            this->execute_instr = INSTR_CSRRW_EX ;
            break;
        case   FUNC3_INSTR_CSRRS  :
            //this->execute_cb =instr_CSRRS;
            this->execute_instr = INSTR_CSRRS_EX ;
            break;
        case   FUNC3_INSTR_CSRRC  :
          //  this->execute_cb =instr_CSRRC;
            this->execute_instr = INSTR_CSRRC_EX ;
            break;
        case   FUNC3_INSTR_CSRRWI  :
           // this->execute_cb =instr_CSRRWI;
            this->execute_instr = INSTR_CSRRWI_EX ;
            break;
        case   FUNC3_INSTR_CSRRSI  :
           // this->execute_cb =instr_CSRRSI;
            this->execute_instr = INSTR_CSRRSI_EX ;
            break;
        case   FUNC3_INSTR_CSRRCI  :
            //this->execute_cb =instr_CSRRCI;
            this->execute_instr = INSTR_CSRRCI_EX ;
            break;
        case   FUNC3_INSTR_ECALL_EBREAK_MRET_SRET_URET_WFI_SFENCEVMA  :
        {
            preparation_func7(&next_subcode2) ;

            switch (next_subcode2 )
            {
            case FUNC7_INSTR_ECALL_EBREAK_URET :
            {
                preparation_func7_func12_sub5_extended  (&next_subcode3) ;


                switch (next_subcode3 )
                {
                case FUNC5_INSTR_ECALL :
                {
                   // this->execute_cb = instr_ECALL ;
                    this->execute_instr = INSTR_ECALL_EX ;
                    break ;
                }
                case FUNC5_INSTR_EBREAK :
                {
                   // this->execute_cb = instr_EBREAK ;
                    this->execute_instr = INSTR_EBREAK_EX ;
                    break ;
                }
                case FUNC5_INSTR_URET :
                {
                   // this->execute_cb = instr_URET ;
                    this->execute_instr = INSTR_URET_EX ;
                    break ;
                }
                }


                break ;
            }
            case FUNC7_INSTR_SRET_WFI :
            {
                preparation_func7_func12_sub5_extended  (&next_subcode3) ;

                switch (next_subcode3 )
                {
                case FUNC5_INSTR_SRET :
                {
                    //this->execute_cb = instr_SRET ;
                    this->execute_instr = INSTR_SRET_EX ;
                    break ;
                }
                case FUNC5_INSTR_WFI :
                {
                    //this->execute_cb = instr_NOP ;
                    this->execute_instr = INSTR_NOP_EX ;
                    break ;
                }

                }

                break ;
            }
            case FUNC7_INSTR_MRET :
            {
                //this->execute_cb = instr_MRET ;
                this->execute_instr = INSTR_MRET_EX ;
                break ;
            }
            case FUNC7_INSTR_SFENCEVMA :
            {
                //this->execute_cb = instr_NOP ;
                this->execute_instr = INSTR_NOP_EX ;
                break ;
            }
            }
            break;
        }

        default:
            break;
        }

        break ;
    }

#endif

#ifdef ATOMIC_SUPPORT

        //D_LR_SC_SWAP_ADD_XOR_AND_OR_MIN_MAX_MINU_MAXU_func5_subcode_list_desc
    case INSTR_AMO_W_D_LR_SC_SWAP_ADD_XOR_AND_OR_MIN_MAX_MINU_MAXU :
    {

        int32_t next_subcode = -1;
        R_type_preparation( &next_subcode);
        int32_t next_subcode2 = -1;
        switch ( next_subcode )
        {

        case   FUNC3_INSTR_W_LR_SC_SWAP_ADD_XOR_AND_OR_MIN_MAX_MINU_MAXU  :
        {
            preparation_func5(&next_subcode2) ;

            switch (next_subcode2 )
            {
            case FUNC5_INSTR_AMO_LR :
            {
               // this->execute_cb = instr_LR_W ;
                this->execute_instr = INSTR_AMO_LR_EX ;
                break ;
            }

            case FUNC5_INSTR_AMO_SC :
            {
                //this->execute_cb = instr_SC_W ;
                this->execute_instr = INSTR_AMO_SC_EX ;
                break ;
            }
            case FUNC5_INSTR_AMO_SWAP :
            {
               // this->execute_cb = instr_AMOSWAP_W ;
                this->execute_instr = INSTR_AMO_SWAP_EX ;
                break ;
            }
            case FUNC5_INSTR_AMO_ADD :
            {
                //this->execute_cb = instr_AMOADD_W ;
                this->execute_instr = INSTR_AMO_ADD_EX ;
                break ;
            }
            case FUNC5_INSTR_AMO_XOR :
            {
               // this->execute_cb = instr_AMOXOR_W ;
                this->execute_instr = INSTR_AMO_XOR_EX ;
                break ;
            }
            case FUNC5_INSTR_AMO_AND :
            {
                //this->execute_cb = instr_AMOAND_W ;
                this->execute_instr = INSTR_AMO_AND_EX ;
                break ;
            }
            case FUNC5_INSTR_AMO_OR :
            {
               // this->execute_cb = instr_AMOOR_W ;
                this->execute_instr = INSTR_AMO_OR_EX ;
                break ;
            }
            case FUNC5_INSTR_AMO_MIN :
            {
               // this->execute_cb = instr_AMOMIN_W ;
                this->execute_instr = INSTR_AMO_MIN_EX ;
                break ;
            }

            case FUNC5_INSTR_AMO_MAX :
            {
              //  this->execute_cb = instr_AMOMAX_W ;
                this->execute_instr = INSTR_AMO_MAX_EX ;
                break ;
            }
            case FUNC5_INSTR_AMO_MINU :
            {
               // this->execute_cb = instr_AMOMINU_W ;
                this->execute_instr = INSTR_AMO_MINU_EX ;
                break ;
            }
            case FUNC5_INSTR_AMO_MAXU :
            {
              //  this->execute_cb = instr_AMOMAXU_W ;
                this->execute_instr = INSTR_AMO_MAXU_EX ;
                break ;
            }
            }


            break;
        }

        default:
            break;
        }

        break ;
    }
#endif


    default :
        printf ("ERROR");
     //   rv_call_from_opcode_list(rv_core, &RV_opcode_list_desc, this->opcode);
    }

    return 0;
}

 rv_uint_xlen rv_core_class::rv_core_execute()
{

    switch (this->execute_instr )
    {
    case INSTR_NOP_EX :
        instr_NOP();
                break;
    case INSTR_ADD_EX :
        instr_ADD();
                break;
    case INSTR_MUL_EX :
        instr_MUL();
                break;

    case INSTR_SUB_EX:
        instr_SUB();
                break;
    case INSTR_SLL_EX:
        instr_SLL();
                break;
    case INSTR_MULH_EX:
         instr_MULH();
                break;
    case INSTR_SLT_EX:
        instr_SLT();
                break;
    case INSTR_MULHSU_EX:
        instr_MULHSU();
                break;
    case INSTR_SLTU_EX:
        instr_SLTU();
                break;
    case INSTR_MULHU_EX:
        instr_MULHU();
                break;
    case INSTR_XOR_EX :
        instr_XOR();
                break;
    case INSTR_DIV_EX:
        instr_DIV();
                break;
    case INSTR_SRL_EX:
        instr_SRL();
                break;
    case INSTR_SRA_EX:
        instr_SRA();
                break;
    case INSTR_DIVU_EX:
        instr_DIVU();
                break;
    case INSTR_OR_EX:
        instr_OR();
                break;
    case INSTR_REM_EX:
         instr_REM();
                break;
    case INSTR_AND_EX:
        instr_AND();
                break;
    case INSTR_REMU_EX:
        instr_REMU();
                break;

    /* I-Type Instructions */
    case INSTR_JALR_EX:
        instr_JALR();
        break;
    case INSTR_ADDI_EX:
        instr_ADDI();
                break;
    case INSTR_SLTI_EX:
        instr_SLTI();
                break;
    case INSTR_SLTIU_EX:
        instr_SLTIU();
                break;
    case INSTR_XORI_EX:
        instr_XORI();
                break;
    case INSTR_ORI_EX:
        instr_ORI();
                break;
    case INSTR_ANDI_EX:
        instr_ANDI();
                break;
    case INSTR_SLLI_EX:
        instr_SLLI();
                break;
    case INSTR_SRLI_F7_EX:
        instr_SRLI();
                break;
    case INSTR_SRAI_F7_EX:
        instr_SRAI();
                break;
    case INSTR_SRLI_F6_EX:
        instr_SRLI();
                break;
    case INSTR_SRAI_F6_EX:
        instr_SRAI();
                break;

     case INSTR_LB_EX:
        instr_LB();
                break;
    case INSTR_LH_EX:
        instr_LH();
                break;
    case INSTR_LW_EX:
        instr_LW();
                break;
    case INSTR_LBU_EX:
        instr_LBU();
                break;
    case INSTR_LHU_EX:
        instr_LHU();
                break;
  //    case INSTR_LWU_EX:
  //      instr_LWU(rv_core);
  //              break;
  //  case INSTR_LD_EX:
  //      instr_LD(rv_core);
  //              break;

    /* S-Type Instructions */

    case INSTR_SB_EX:
        instr_SB();
                break;
    case INSTR_SH_EX:
        instr_SH();
                break;
     case INSTR_SW_EX:
       instr_SW();
                break;
     // case INSTR_SD_EX:
     //   instr_SD(rv_core);
      //          break;

    /* B-Type Instructions */



    case INSTR_BEQ_EX:
            instr_BEQ();
        break ;
    case INSTR_BNE_EX:
            instr_BNE();
        break ;
    case INSTR_BLT_EX:
        instr_BLT();
        break;
    case INSTR_BGE_EX:
        instr_BGE();
        break;
    case INSTR_BLTU_EX:
        instr_BLTU();
        break;

    case INSTR_BGEU_EX:
        instr_BGEU();
        break;
    /* U-Type Instructions */
     case INSTR_LUI_EX:
        instr_LUI();
                break;
    case INSTR_AUIPC_EX:
        instr_AUIPC();
                break;

    /* J-Type Instructions */
    case INSTR_JAL_EX:
        instr_JAL ();
        break ;
    /* System level instructions */
    case INSTR_FENCE_EX:
        instr_NOP();
                break;
    case INSTR_FENCE_I_EX:
        instr_NOP();
                break;


    case INSTR_ECALL_EX:
        instr_ECALL();
                break;
    case INSTR_EBREAK_EX:
        instr_EBREAK();
                break;
    case INSTR_URET_EX:
         instr_URET ();
                break;
    case INSTR_SRET_EX:
        instr_SRET();
                break;
    case INSTR_WFI_EX:
        instr_NOP();
                break;
    case INSTR_MRET_EX:
        instr_MRET();
                break;
    case INSTR_SFENCEVMA_EX:
        instr_NOP();
                break;
    case INSTR_CSRRW_EX :
        instr_CSRRW();
                break;
    case INSTR_CSRRS_EX :
        instr_CSRRS();
                break;
    case INSTR_CSRRC_EX :
        instr_CSRRC();
                break;
    case INSTR_CSRRWI_EX:
        instr_CSRRWI();
                break;
    case INSTR_CSRRSI_EX:
        instr_CSRRSI();
                break;
    case INSTR_CSRRCI_EX:
        instr_CSRRCI();
                break;

   // case INSTR_SLLIW_EX:
   //     instr_SLLIW(rv_core);
   //             break;
//    case INSTR_SRLIW_EX:
   //     (rv_core);
   //             break;
//    case INSTR_SRAIW_EX:
   //     (rv_core);
   //             break;
  //  case INSTR_ADDIW_EX:
  //   instr_ADDIW   (rv_core);
      //          break;

//    case INSTR_ADDW_EX:
//        instr_ADDW(rv_core);
 //               break;
//    case INSTR_SUBW_EX:
   //     (rv_core);
   //             break;
//    case INSTR_MULW_EX:
   //     (rv_core);
   //             break;
//    case INSTR_SLLW_EX:
   //     (rv_core);
   //             break;
//    case INSTR_DIVW_EX:
   //     (rv_core);
   //             break;

//    case INSTR_SRLW_EX:
   //     (rv_core);
   //             break;
//    case INSTR_SRAW_EX:
   //     (rv_core);
   //             break;
//    case INSTR_DIVUW_EX:
   //     (rv_core);
   //             break;
//    case INSTR_REMW_EX:
   //     (rv_core);
                break;
//    case INSTR_REMUW_EX:
   //     (rv_core);
   //             break;



//    /* Atomic Instructions */
    case INSTR_AMO_LR_EX:
        instr_LR_W();
                break;
    case INSTR_AMO_SC_EX:
        instr_SC_W();
                break;
    case INSTR_AMO_SWAP_EX:
        instr_AMOSWAP_W();
                break;
    case INSTR_AMO_ADD_EX:
        instr_AMOADD_W();
                break;
    case INSTR_AMO_XOR_EX:
        instr_AMOXOR_W();
                break;
    case INSTR_AMO_AND_EX:
        instr_AMOAND_W();
                break;
    case INSTR_AMO_OR_EX:
        instr_AMOOR_W();
                break;
    case INSTR_AMO_MIN_EX:
        instr_AMOMIN_W();
                break;
    case INSTR_AMO_MAX_EX:
        instr_AMOMAX_W();
                break;
    case INSTR_AMO_MINU_EX:
        instr_AMOMINU_W();
                break;
    case INSTR_AMO_MAXU_EX:
        instr_AMOMAXU_W();
                break;

    default :
        break ;
            //this->execute_cb(rv_core);

    }
    //this->execute_cb(rv_core);

    /* clear x0 if any instruction has written into it */
    this->x[0] = 0;

    return 0;
}

/******************* Public functions *******************************/
void rv_core_class::rv_core_run(/*rv_core_td *rv_core*/)
{
    this->next_pc = 0;

    if(rv_core_fetch() == rv_ok)
    {
        rv_core_decode();
        rv_core_execute();
    }

    /* increase program counter here */
    this->pc = this->next_pc ? this->next_pc : this->pc + 4;

    this->curr_cycle++;
    this->csr_regs[CSR_ADDR_MCYCLE].value = this->curr_cycle;
    this->csr_regs[CSR_ADDR_MINSTRET].value = this->curr_cycle;
    this->csr_regs[CSR_ADDR_CYCLE].value = this->curr_cycle;
    this->csr_regs[CSR_ADDR_TIME].value = this->curr_cycle;

#ifndef RV64
    this->csr_regs[CSR_ADDR_MCYCLEH].value = this->curr_cycle >> 32;
    this->csr_regs[CSR_ADDR_MINSTRETH].value = this->curr_cycle >> 32;
    this->csr_regs[CSR_ADDR_CYCLEH].value = this->curr_cycle >> 32;
    this->csr_regs[CSR_ADDR_TIMEH].value = this->curr_cycle >> 32;
#endif
}

void rv_core_class::rv_core_process_interrupts(/*rv_core_td *rv_core,*/ uint8_t mei, uint8_t mti, uint8_t msi)
{
#ifdef CSR_SUPPORT
    /* interrupt handling */
    rv_core_update_interrupts( mei, mti, msi);
    rv_core_prepare_interrupts();
#else
    (void)rv_core;
    (void)mei;
    (void)msi;
    (void)mti;
#endif
}

void rv_core_class::rv_core_reg_dump(/*rv_core_td *rv_core*/)
{
 //   (void) rv_core;

    int i = 0;

    DEBUG_PRINT("pc: " PRINTF_FMT "\n", this->pc);
    DEBUG_PRINT("instr: %08x\n", this->instruction);
    for(i=0;i<NR_RVI_REGS;i++)
    {
        DEBUG_PRINT("x[%2d]: " PRINTF_FMT "\n", i, this->x[i]);
    }
}

void rv_core_class::rv_core_reg_dump_more_regs(/*rv_core_td *rv_core*/)
{
//    (void) rv_core;

    DEBUG_PRINT("internal regs after execution:\n");
    DEBUG_PRINT("instruction: %x\n", this->instruction);
    DEBUG_PRINT("rd: %x rs1: %x rs2: %x imm: " PRINTF_FMT "\n", this->rd, this->rs1, this->rs2, this->immediate);
    DEBUG_PRINT("func3: %x func7: %x jump_offset " PRINTF_FMT "\n", this->func3, this->func7, this->jump_offset);
    DEBUG_PRINT("next pc: " PRINTF_FMT "\n", this->pc);
    DEBUG_PRINT("\n");
}

 void rv_core_class::rv_core_init_csr_regs()
{
    uint16_t i = 0;

    /* Machine Information Registers */
    INIT_CSR_REG_DEFAULT(this->csr_regs, CSR_ADDR_MVENDORID, CSR_ACCESS_RO(machine_mode), 0, CSR_MASK_ZERO);
    INIT_CSR_REG_DEFAULT(this->csr_regs, CSR_ADDR_MARCHID, CSR_ACCESS_RO(machine_mode), 0, CSR_MASK_ZERO);
    INIT_CSR_REG_DEFAULT(this->csr_regs, CSR_ADDR_MIMPID, CSR_ACCESS_RO(machine_mode), 0, CSR_MASK_ZERO);
    INIT_CSR_REG_DEFAULT(this->csr_regs, CSR_ADDR_MHARTID, CSR_ACCESS_RO(machine_mode), 0, CSR_MASK_ZERO);

    /* Machine Trap Setup */
    INIT_CSR_REG_SPECIAL(this->csr_regs, CSR_ADDR_MSTATUS, CSR_ACCESS_RW(machine_mode), CSR_MSTATUS_MASK, &this->trap, trap_m_read, trap_m_write, trap_reg_status);
    INIT_CSR_REG_SPECIAL(this->csr_regs, CSR_ADDR_MISA, CSR_ACCESS_RO(machine_mode), CSR_MASK_WR_ALL, &this->trap, trap_m_read, trap_m_write, trap_reg_isa);
    INIT_CSR_REG_SPECIAL(this->csr_regs, CSR_ADDR_MEDELEG, CSR_ACCESS_RW(machine_mode), CSR_MEDELEG_MASK, &this->trap, trap_m_read, trap_m_write, trap_reg_edeleg);
    INIT_CSR_REG_SPECIAL(this->csr_regs, CSR_ADDR_MIDELEG, CSR_ACCESS_RW(machine_mode), CSR_MIDELEG_MASK, &this->trap, trap_m_read, trap_m_write, trap_reg_ideleg);
    INIT_CSR_REG_SPECIAL(this->csr_regs, CSR_ADDR_MIE, CSR_ACCESS_RW(machine_mode), CSR_MIP_MIE_MASK, &this->trap, trap_m_read, trap_m_write, trap_reg_ie);
    INIT_CSR_REG_SPECIAL(this->csr_regs, CSR_ADDR_MTVEC, CSR_ACCESS_RW(machine_mode), CSR_MTVEC_MASK, &this->trap, trap_m_read, trap_m_write, trap_reg_tvec);
    INIT_CSR_REG_DEFAULT(this->csr_regs, CSR_ADDR_MCOUNTEREN, CSR_ACCESS_RW(machine_mode), 0, CSR_MASK_ZERO);

    /* Machine Trap Handling */
    INIT_CSR_REG_SPECIAL(this->csr_regs, CSR_ADDR_MSCRATCH, CSR_ACCESS_RW(machine_mode), CSR_MASK_WR_ALL, &this->trap, trap_m_read, trap_m_write, trap_reg_scratch);
    INIT_CSR_REG_SPECIAL(this->csr_regs, CSR_ADDR_MEPC, CSR_ACCESS_RW(machine_mode), CSR_MASK_WR_ALL, &this->trap, trap_m_read, trap_m_write, trap_reg_epc);
    INIT_CSR_REG_SPECIAL(this->csr_regs, CSR_ADDR_MCAUSE, CSR_ACCESS_RW(machine_mode), CSR_MASK_WR_ALL, &this->trap, trap_m_read, trap_m_write, trap_reg_cause);
    INIT_CSR_REG_SPECIAL(this->csr_regs, CSR_ADDR_MTVAL, CSR_ACCESS_RW(machine_mode), CSR_MASK_WR_ALL, &this->trap, trap_m_read, trap_m_write, trap_reg_tval);
    INIT_CSR_REG_SPECIAL(this->csr_regs, CSR_ADDR_MIP, CSR_ACCESS_RW(machine_mode), CSR_MIP_MIE_MASK, &this->trap, trap_m_read, trap_m_write, trap_reg_ip);

    /* Set supported ISA Extension bits */
    *this->trap.m.regs[trap_reg_isa] = RV_SUPPORTED_EXTENSIONS;
#ifdef RV64
    *this->trap.m.regs[trap_reg_isa] |= (2UL << (XLEN-2));
#else
    *this->trap.m.regs[trap_reg_isa] |= (1 << (XLEN-2));
#endif

    /* Machine Protection and Translation */
    for(i=0;i<PMP_NR_CFG_REGS;i++)
    {
#ifdef PMP_SUPPORT
        INIT_CSR_REG_SPECIAL(this->csr_regs, (CSR_PMPCFG0+i), CSR_ACCESS_RW(machine_mode), CSR_MASK_WR_ALL, &this->pmp, pmp_read_csr_cfg, pmp_write_csr_cfg, i);
#else
        INIT_CSR_REG_DEFAULT(this->csr_regs, (CSR_PMPCFG0+i), CSR_ACCESS_RW(machine_mode), 0, CSR_MASK_WR_ALL);
#endif
    }

    /* All others are WARL */
    for(i=PMP_NR_CFG_REGS;i<PMP_NR_CFG_REGS_WARL_MAX;i++)
        INIT_CSR_REG_DEFAULT(this->csr_regs, (CSR_PMPCFG0+i), CSR_ACCESS_RO(machine_mode), 0, CSR_MASK_ZERO);

    for(i=0;i<PMP_NR_ADDR_REGS;i++)
    {
#ifdef PMP_SUPPORT
        INIT_CSR_REG_SPECIAL(this->csr_regs, (CSR_PMPADDR0+i), CSR_ACCESS_RW(machine_mode), CSR_MASK_WR_ALL, &this->pmp, pmp_read_csr_addr, pmp_write_csr_addr, i);
#else
        INIT_CSR_REG_DEFAULT(this->csr_regs, (CSR_PMPADDR0+i), CSR_ACCESS_RW(machine_mode), 0, CSR_MASK_WR_ALL);
#endif
    }

    /* All others are WARL */
    for(i=PMP_NR_ADDR_REGS;i<PMP_NR_ADDR_REGS_WARL_MAX;i++)
        INIT_CSR_REG_DEFAULT(this->csr_regs, (CSR_PMPADDR0+i), CSR_ACCESS_RO(machine_mode), 0, CSR_MASK_ZERO);

    /* Supervisor Trap Setup */
    INIT_CSR_REG_SPECIAL(this->csr_regs, CSR_ADDR_SSTATUS, CSR_ACCESS_RW(machine_mode) | CSR_ACCESS_RW(supervisor_mode), CSR_SSTATUS_MASK, &this->trap, trap_s_read, trap_s_write, trap_reg_status);
    INIT_CSR_REG_SPECIAL(this->csr_regs, CSR_ADDR_SEDELEG, CSR_ACCESS_RW(machine_mode) | CSR_ACCESS_RW(supervisor_mode), CSR_SEDELEG_MASK, &this->trap, trap_s_read, trap_s_write, trap_reg_edeleg);
    INIT_CSR_REG_SPECIAL(this->csr_regs, CSR_ADDR_SIDELEG, CSR_ACCESS_RW(machine_mode) | CSR_ACCESS_RW(supervisor_mode), CSR_SIDELEG_MASK, &this->trap, trap_s_read, trap_s_write, trap_reg_ideleg);
    INIT_CSR_REG_SPECIAL(this->csr_regs, CSR_ADDR_SIE, CSR_ACCESS_RW(machine_mode) | CSR_ACCESS_RW(supervisor_mode), CSR_SIP_SIE_MASK, &this->trap, trap_s_read, trap_s_write, trap_reg_ie);
    INIT_CSR_REG_SPECIAL(this->csr_regs, CSR_ADDR_STVEC, CSR_ACCESS_RW(machine_mode) | CSR_ACCESS_RW(supervisor_mode), CSR_STVEC_MASK, &this->trap, trap_s_read, trap_s_write, trap_reg_tvec);
    INIT_CSR_REG_DEFAULT(this->csr_regs, CSR_ADDR_SCOUNTEREN, CSR_ACCESS_RW(machine_mode) | CSR_ACCESS_RW(supervisor_mode), 0, CSR_MASK_ZERO);

    /* Supervisor Trap Setup */
    INIT_CSR_REG_SPECIAL(this->csr_regs, CSR_ADDR_SSCRATCH, CSR_ACCESS_RW(machine_mode) | CSR_ACCESS_RW(supervisor_mode), CSR_MASK_WR_ALL, &this->trap, trap_s_read, trap_s_write, trap_reg_scratch);
    INIT_CSR_REG_SPECIAL(this->csr_regs, CSR_ADDR_SEPC, CSR_ACCESS_RW(machine_mode) | CSR_ACCESS_RW(supervisor_mode), CSR_MASK_WR_ALL, &this->trap, trap_s_read, trap_s_write, trap_reg_epc);
    INIT_CSR_REG_SPECIAL(this->csr_regs, CSR_ADDR_SCAUSE, CSR_ACCESS_RW(machine_mode) | CSR_ACCESS_RW(supervisor_mode), CSR_MASK_WR_ALL, &this->trap, trap_s_read, trap_s_write, trap_reg_cause);
    INIT_CSR_REG_SPECIAL(this->csr_regs, CSR_ADDR_STVAL, CSR_ACCESS_RW(machine_mode) | CSR_ACCESS_RW(supervisor_mode), CSR_MASK_WR_ALL, &this->trap, trap_s_read, trap_s_write, trap_reg_tval);
    INIT_CSR_REG_SPECIAL(this->csr_regs, CSR_ADDR_SIP, CSR_ACCESS_RW(machine_mode) | CSR_ACCESS_RW(supervisor_mode), CSR_SIP_SIE_MASK, &this->trap, trap_s_read, trap_s_write, trap_reg_ip);

    /* Supervisor Address Translation and Protection */
    INIT_CSR_REG_SPECIAL(this->csr_regs, CSR_ADDR_SATP, CSR_ACCESS_RW(machine_mode) | CSR_ACCESS_RW(supervisor_mode), CSR_SATP_MASK, &this->mmu, mmu_read_csr, mmu_write_csr, 0);

    /* Performance Counters */
    INIT_CSR_REG_DEFAULT(this->csr_regs, (CSR_ADDR_MCYCLE), CSR_ACCESS_RW(machine_mode), 0, CSR_MASK_WR_ALL);
    INIT_CSR_REG_DEFAULT(this->csr_regs, (CSR_ADDR_MCYCLEH), CSR_ACCESS_RW(machine_mode), 0, CSR_MASK_WR_ALL);
    INIT_CSR_REG_DEFAULT(this->csr_regs, (CSR_ADDR_MINSTRET), CSR_ACCESS_RW(machine_mode), 0, CSR_MASK_WR_ALL);
    INIT_CSR_REG_DEFAULT(this->csr_regs, (CSR_ADDR_MINSTRETH), CSR_ACCESS_RW(machine_mode), 0, CSR_MASK_WR_ALL);

    INIT_CSR_REG_DEFAULT(this->csr_regs, (CSR_ADDR_CYCLE), CSR_ACCESS_RO(machine_mode) | CSR_ACCESS_RO(supervisor_mode) | CSR_ACCESS_RO(user_mode), 0, CSR_MASK_WR_ALL);
    INIT_CSR_REG_DEFAULT(this->csr_regs, (CSR_ADDR_CYCLEH), CSR_ACCESS_RO(machine_mode) | CSR_ACCESS_RO(supervisor_mode) | CSR_ACCESS_RO(user_mode), 0, CSR_MASK_WR_ALL);
    INIT_CSR_REG_DEFAULT(this->csr_regs, (CSR_ADDR_TIME), CSR_ACCESS_RO(machine_mode) | CSR_ACCESS_RO(supervisor_mode) | CSR_ACCESS_RO(user_mode), 0, CSR_MASK_WR_ALL);
    INIT_CSR_REG_DEFAULT(this->csr_regs, (CSR_ADDR_TIMEH), CSR_ACCESS_RO(machine_mode) | CSR_ACCESS_RO(supervisor_mode) | CSR_ACCESS_RO(user_mode), 0, CSR_MASK_WR_ALL);

    /* All others are WARL, they start at 3 */
    for(i=3;i<CSR_HPMCOUNTER_WARL_MAX;i++)
        INIT_CSR_REG_DEFAULT(this->csr_regs, (CSR_ADDR_MCYCLE+i), CSR_ACCESS_RO(machine_mode), 0, CSR_MASK_ZERO);
    INIT_CSR_REG_DEFAULT(this->csr_regs, (CSR_ADDR_CYCLE+i), CSR_ACCESS_RO(machine_mode) | CSR_ACCESS_RO(supervisor_mode) | CSR_ACCESS_RO(user_mode), 0, CSR_MASK_ZERO);

    for(i=3;i<CSR_HPMCOUNTER_WARL_MAX;i++)
        INIT_CSR_REG_DEFAULT(this->csr_regs, (CSR_ADDR_MCYCLEH+i), CSR_ACCESS_RW(machine_mode), 0, CSR_MASK_ZERO);
    INIT_CSR_REG_DEFAULT(this->csr_regs, (CSR_ADDR_CYCLEH+i), CSR_ACCESS_RO(machine_mode) | CSR_ACCESS_RO(supervisor_mode) | CSR_ACCESS_RO(user_mode), 0, CSR_MASK_ZERO);
}

void rv_core_class::rv_core_init(/*rv_core_td *rv_core,*/
                  void *priv,
                  bus_access_func bus_access
                  )
{
    memset(this, 0, sizeof(rv_core_td));

    this->curr_priv_mode = machine_mode;
    this->pc = MROM_BASE_ADDR;

    this->priv = priv;
    this->bus_access = bus_access;

    trap_init(&this->trap);
    mmu_init(&this->mmu, pmp_checked_bus_access, this);

    rv_core_init_csr_regs();
}


/***************************/

