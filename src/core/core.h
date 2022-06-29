#ifndef RISCV_CORE_H
#define RISCV_CORE_H

#include <riscv_types.h>
#include <riscv_instr.h>

#include <csr.h>
#include <pmp.h>
#include <trap.h>
#include <clint.h>

#define NR_RVI_REGS 32

//typedef struct rv_core_struct rv_core_td;
typedef class  rv_core_class rv_core_td;

#include <mmu.h>

//typedef struct rv_core_struct
//{
//    privilege_level curr_priv_mode;
//    uint64_t curr_cycle;

//    /* Registers */
//    rv_uint_xlen x[NR_RVI_REGS];
//    rv_uint_xlen pc;
//    rv_uint_xlen next_pc;

//    uint32_t instruction;
//    uint8_t opcode;
//    uint8_t rd;
//    uint8_t rs1;
//    uint8_t rs2;
//    uint8_t func3;
//    uint8_t func7;
//    uint8_t func6;
//    uint8_t func5;
//    uint16_t func12;
//    rv_uint_xlen immediate;
//    rv_uint_xlen jump_offset;

//    uint8_t sync_trap_pending;
//    rv_uint_xlen sync_trap_cause;
//    rv_uint_xlen sync_trap_tval;

//    /* points to the next instruction */
//   // void (*execute_cb)(rv_core_td *rv_core);

//    instr_to_execute execute_instr;
//    /* externally hooked */
//    void *priv;
//    bus_access_func bus_access;
//    // bus_read_mem read_mem;
//    // bus_write_mem write_mem;

//    csr_reg_td csr_regs[CSR_ADDR_MAX];
//    pmp_td pmp;
//    trap_td trap;
//    mmu_td mmu;

//    int lr_valid;
//    rv_uint_xlen lr_address;

//} rv_core_td;




typedef class  rv_core_class
{
public:
    privilege_level curr_priv_mode;
    uint64_t curr_cycle;

    /* Registers */
    rv_uint_xlen x[NR_RVI_REGS];
    rv_uint_xlen pc;
    rv_uint_xlen next_pc;

    uint32_t instruction;
    uint8_t opcode;
    uint8_t rd;
    uint8_t rs1;
    uint8_t rs2;
    uint8_t func3;
    uint8_t func7;
    uint8_t func6;
    uint8_t func5;
    uint16_t func12;
    rv_uint_xlen immediate;
    rv_uint_xlen jump_offset;

    uint8_t sync_trap_pending;
    rv_uint_xlen sync_trap_cause;
    rv_uint_xlen sync_trap_tval;

    /* points to the next instruction */
   // void (*execute_cb)(rv_core_td *rv_core);

    instr_to_execute execute_instr;
    /* externally hooked */
    void *priv;
    bus_access_func bus_access;
    // bus_read_mem read_mem;
    // bus_write_mem write_mem;

    csr_reg_td csr_regs[CSR_ADDR_MAX];
    pmp_td pmp;
    trap_td trap;
    mmu_td mmu;

    int lr_valid;
    rv_uint_xlen lr_address;

  //  void run();
  //  void process_interrupts( uint8_t mei, uint8_t mti, uint8_t msi);
  //  void init(bus_access_func bus_access);

    void rv_core_run(/*rv_core_td *rv_core*/);
    void rv_core_process_interrupts(/*rv_core_td *rv_core,*/ uint8_t mei, uint8_t mti, uint8_t msi);
    void rv_core_reg_dump(/*rv_core_td *rv_core*/);
    void rv_core_reg_dump_more_regs(/*rv_core_td *rv_core*/);
    void rv_core_init(/*rv_core_td *rv_core,*/
                      void *priv,
                      bus_access_func bus_access
                      );
    inline void prepare_sync_trap( rv_uint_xlen cause, rv_uint_xlen tval) ;
    inline privilege_level check_mprv_override( bus_access_type access_type) ;
private:
    void rv_core_init_csr_regs();


   //rv_ret pmp_checked_bus_access(void *priv, privilege_level priv_level, bus_access_type access_type, rv_uint_xlen addr, void *value, uint8_t len) ;
   //rv_ret mmu_checked_bus_access(void *priv, privilege_level priv_level, bus_access_type access_type, rv_uint_xlen addr, void *value, uint8_t len) ;
    void instr_NOP() ;
    void instr_LUI() ;
    void instr_AUIPC() ;
    void instr_JAL() ;
    void instr_JALR() ;
    void instr_BEQ() ;
    void instr_BNE() ;
    void instr_BLT()                   ;
    void instr_BGE()                   ;
    void instr_BLTU()                  ;
    void instr_BGEU()                  ;
    void instr_ADDI()                  ;
    void instr_SLTI()                  ;
    void instr_SLTIU()                 ;
    void instr_XORI()                  ;
    void instr_ORI()                   ;
    void instr_ANDI()                  ;
    void instr_SLLI()                  ;
    void instr_SRAI()                  ;
    void instr_SRLI()                  ;
    void instr_ADD()                   ;
    void instr_SUB()                   ;
    void instr_SLL()                   ;
    void instr_SLT()                   ;
    void instr_SLTU()                  ;
    void instr_XOR()                   ;
    void instr_SRL()                   ;
    void instr_OR()                    ;
    void instr_AND()                   ;
    void instr_SRA()                   ;
    void instr_LB()                    ;
    void instr_LH()                    ;
    void instr_LW()                    ;
    void instr_LBU()                   ;
    void instr_LHU()                   ;
    void instr_SB()                    ;
    void instr_SH()                    ;
    void instr_SW()                    ;
    inline void CSRRWx( rv_uint_xlen new_val)      ;
    inline void CSRRSx( rv_uint_xlen new_val)      ;
    inline void CSRRCx( rv_uint_xlen new_val)      ;
    void instr_CSRRW()                            ;
    void instr_CSRRS()                            ;
    void instr_CSRRC()                            ;
    void instr_CSRRWI()                           ;
    void instr_CSRRSI()                           ;
    void instr_CSRRCI()                           ;
    void instr_ECALL()                            ;
    void instr_EBREAK()                           ;
    void instr_MRET()                             ;
    void instr_SRET()                             ;
    void instr_URET()                             ;
    void instr_LR_W()                             ;
    void instr_SC_W()                             ;
    void instr_AMOSWAP_W()                        ;
    void instr_AMOADD_W()                         ;
    void instr_AMOXOR_W()                         ;
    void instr_AMOAND_W()                         ;
    void instr_AMOOR_W()                          ;
    void instr_AMOMIN_W()                         ;
    void instr_AMOMAX_W()                         ;
    void instr_AMOMINU_W()                        ;
    void instr_AMOMAXU_W()                        ;
    void instr_DIV()                              ;
    void instr_DIVU()                             ;
    void instr_REM()                              ;
    void instr_REMU()                             ;
    void instr_MUL()                              ;
    void instr_MULH()                             ;
    void instr_MULHU()                            ;
    void instr_MULHSU()                           ;
    void preparation_func5( int32_t *next_subcode) ;
    void preparation_func7( int32_t *next_subcode) ;
    void preparation_func7_func12_sub5_extended( int32_t *next_subcode) ;
    void R_type_preparation( int32_t *next_subcode) ;
    void I_type_preparation( int32_t *next_subcode) ;
    void S_type_preparation( int32_t *next_subcode) ;
    void B_type_preparation( int32_t *next_subcode) ;
    void U_type_preparation( int32_t *next_subcode) ;
    void J_type_preparation( int32_t *next_subcode) ;
    inline void rv_core_update_interrupts( uint8_t mei, uint8_t mti, uint8_t msi) ;
    inline uint8_t rv_core_prepare_interrupts() ;
    inline rv_uint_xlen rv_core_fetch() ;
    inline rv_uint_xlen rv_core_decode() ;
    rv_uint_xlen rv_core_execute() ;


} rv_core_td ;
/*
void rv_core_run(rv_core_td *rv_core);
void rv_core_process_interrupts(rv_core_td *rv_core, uint8_t mei, uint8_t mti, uint8_t msi);
void rv_core_reg_dump(rv_core_td *rv_core);
void rv_core_reg_dump_more_regs(rv_core_td *rv_core);
void rv_core_init(rv_core_td *rv_core,
                  void *priv,
                  bus_access_func bus_access
                  );
*/
//typedef struct instruction_hook_struct
//{
//    void (*preparation_cb)(rv_core_td *rv_core, int32_t *next_subcode);
//    void (*execution_cb)(rv_core_td *rv_core_data);
//    struct instruction_desc_struct *next;

//} instruction_hook_td;

//typedef struct instruction_desc_struct
//{
//    unsigned int instruction_hook_list_size;
//    instruction_hook_td *instruction_hook_list;

//} instruction_desc_td;
//#define INIT_INSTRUCTION_LIST_DESC(_instruction_list) \
//    static instruction_desc_td  _instruction_list##_desc = \
//    { sizeof(_instruction_list)/sizeof(_instruction_list[0]), _instruction_list }

#endif /* RISCV_CORE_H */
