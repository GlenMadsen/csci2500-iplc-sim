/***********************************************************************/
/***********************************************************************
 Pipeline Cache Simulator
 ***********************************************************************/
/***********************************************************************/
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <math.h>

#define MAX_CACHE_SIZE 10240
#define CACHE_MISS_DELAY 10 // 10 cycle cache miss penalty
#define MAX_STAGES 5

// init the simulator
void iplc_sim_init(int index, int blocksize, int assoc);

// Cache simulator functions
void iplc_sim_LRU_replace_on_miss(int index, int tag);
void iplc_sim_LRU_update_on_hit(int index, int assoc);
int iplc_sim_trap_address(unsigned int address);

// Pipeline functions
unsigned int iplc_sim_parse_reg(char *reg_str);
void iplc_sim_parse_instruction(char *buffer);
void iplc_sim_push_pipeline_stage();
void iplc_sim_process_pipeline_rtype(char *instruction, int dest_reg,
                                     int reg1, int reg2_or_constant);
void iplc_sim_process_pipeline_lw(int dest_reg, int base_reg, unsigned int data_address);
void iplc_sim_process_pipeline_sw(int src_reg, int base_reg, unsigned int data_address);
void iplc_sim_process_pipeline_branch(int reg1, int reg2);
void iplc_sim_process_pipeline_jump();
void iplc_sim_process_pipeline_syscall();
void iplc_sim_process_pipeline_nop();

// Outout performance results
void iplc_sim_finalize();

/* Instead of using a cache_line_t struct, we used a cache_set_t struct to represent an individual
 * associative set, as these variables can be placed in arrays to represent each cache line, and
 * then each cache line into an array to represent the whole cache.
 * valid represent the valid bit, and tag represents the tag. Method for determining order of use
 * is explained in iplc_LRU_replace_on_miss and iplc_LRU_update_on_hit.
 */
typedef struct {
    int valid;
    int tag;
} cache_set_t;

cache_set_t** cache = NULL;
int cache_index=0;
int cache_blocksize=0;
int cache_blockoffsetbits = 0;
int cache_assoc=0;
long cache_miss=0;
long cache_hit=0;

char instruction[16];
char reg1[16];
char reg2[16];
char offsetwithreg[16];
unsigned int data_address=0;
unsigned int instruction_address=0;
unsigned int pipeline_cycles=0;   // how many cycles did you pipeline consume
unsigned int instruction_count=0; // home many real instructions ran thru the pipeline
unsigned int branch_predict_taken=0;
unsigned int branch_count=0;
unsigned int correct_branch_predictions=0;

unsigned int debug=0;
unsigned int dump_pipeline=1;

enum instruction_type {NOP, RTYPE, LW, SW, BRANCH, JUMP, JAL, SYSCALL};

typedef struct rtype
{
    char instruction[16];
    int reg1;
    int reg2_or_constant;
    int dest_reg;
    
} rtype_t;

typedef struct load_word
{
    unsigned int data_address;
    int dest_reg;
    int base_reg;
    
} lw_t;

typedef struct store_word
{
    unsigned int data_address;
    int src_reg;
    int base_reg;
} sw_t;

typedef struct branch
{
    int reg1;
    int reg2;
    
} branch_t;


typedef struct jump
{
    char instruction[16];
    
} jump_t;

typedef struct pipeline
{
    enum instruction_type itype;
    unsigned int instruction_address;
    union
    {
        rtype_t   rtype;
        lw_t      lw;
        sw_t      sw;
        branch_t  branch;
        jump_t    jump;
    }
    stage;
    
} pipeline_t;

enum pipeline_stages {FETCH, DECODE, ALU, MEM, WRITEBACK};

pipeline_t pipeline[MAX_STAGES];

/************************************************************************************************/
/* Cache Functions ******************************************************************************/
/************************************************************************************************/
/*
 * Correctly configure the cache.
 */
void iplc_sim_init(int index, int blocksize, int assoc)
{
    int i=0, j=0;
    unsigned long cache_size = 0;
    cache_index = index;
    cache_blocksize = blocksize;
    cache_assoc = assoc;
    
    
    cache_blockoffsetbits =
    (int) rint((log( (double) (blocksize * 4) )/ log(2)));
    /* Note: rint function rounds the result up prior to casting */
    
    cache_size = assoc * ( 1 << index ) * ((32 * blocksize) + 33 - index - cache_blockoffsetbits);
    
    printf("Cache Configuration \n");
    printf("   Index: %d bits or %d lines \n", cache_index, (1<<cache_index) );
    printf("   BlockSize: %d \n", cache_blocksize );
    printf("   Associativity: %d \n", cache_assoc );
    printf("   BlockOffSetBits: %d \n", cache_blockoffsetbits );
    printf("   CacheSize: %lu \n", cache_size );
    
    if (cache_size > MAX_CACHE_SIZE ) {
        printf("Cache too big. Great than MAX SIZE of %d .... \n", MAX_CACHE_SIZE);
        exit(-1);
    }
    

    /* (See explanation of cache_set_t on line #40) creates the array of arrays of cache_set_t
     * variables and sets each valid bit of each set to 0 to indicate it is empty.
     */
    cache = (cache_set_t**) malloc((sizeof(cache_set_t*) * 1<<index));
    // Dynamically create our cache based on the information the user entered
    for(i = 0; i < (1 << cache_index); i++)
    {
        cache[i] = (cache_set_t*) malloc(sizeof(cache_set_t)*cache_assoc);
        for(j = 0; j < cache_assoc; j++)
        {
            cache[i][j].valid = 0;
        }
    }
    
    // init the pipeline -- set all data to zero and instructions to NOP
    for (i = 0; i < MAX_STAGES; i++) {
        // itype is set to O which is NOP type instruction
        bzero(&(pipeline[i]), sizeof(pipeline_t));
    }
}

/*
 * iplc_sim_trap_address() determined this is not in our cache.  Put it there
 * and make sure that is now our Most Recently Used (MRU) entry.
 */
void iplc_sim_LRU_replace_on_miss(int index, int tag)
{
    /* The cache line has already been found by iplc_sim_trap_address, and the index of the cache
     * line has been passed as index, so cache[index] is the cache line the address needs to be
     * placed in. The least recently used address in the 0th index of cache[index] and the most
     * recently used is kept in the highest valid index. Thus, on a miss, if the last element in
     * the array is not valid, nothing needs to be replaced and the new address can simply be put
     * in the non-valid associative set with the smallest index. However, if the last element in
     * the array is valid, then the array is full and each set is shifted over one in the
     * direction of a lower index, removing the least recently address used from the cache, and
     * then the new address in placed in the associative set with the highest index to indicate
     * that it is the most recently used.
     * E.g. If tags a, b, c, d, e, f, g, and h are in the cache line sorted from least to most
     * recently used:
     *                               a  b  c  d  e  f  g  h
     * and an address with the proper index and tag i misses, then every tag is shifted and i
     * is placed in like so:
     *                               b  c  d  e  f  g  h  i
     * and thus a, the least recently used, is replaced and i becomes the most recently used,
     * while keeping the ordering the same otherwise.
     */
    int i;
    if(!cache[index][cache_assoc-1].valid)
    {
        for(i = 0; i < cache_assoc; i++)
        {
            if(!cache[index][i].valid)
            {
                cache[index][i].tag = tag;
                cache[index][i].valid = 1;
                break;
            }
        }
    }
    else
    {
        for(i = 0; i < cache_assoc-1; i++)
        {
            cache[index][i].tag = cache[index][i+1].tag;
        }
        cache[index][i].tag = tag;
    }
}

/*
 * iplc_sim_trap_address() determined the entry is in our cache.  Update its
 * information in the cache.
 */
void iplc_sim_LRU_update_on_hit(int index, int assoc_entry)
{
    /* (See explanation of LRU method in iplc_sim_LRU_replace_on_miss)
     * On a hit, the address that was found must be updated to reflect that is is now the most
     * recently used. Since assoc_entry has the set it was found in, every valid set after it must
     * be shifted to the left to indicate that it has been used less recently. Then, the set with
     * the found address is placed in either the highest index valid most recently used position.
     * E.g. If tags a, b, c, d, e, f, g, and h are in the cache line sorted from least to most
     * recently used:
     *                               a  b  c  d  e  f  g  h
     * and an address with the proper index and tag c hits, then every tag after c is shifted to
     * the left and c is placed at the end:
     *                               a  b  d  e  f  g  h  c
     * indicating that c is the most recently used while keeping the ordering the same otherwise.
     */
    int i;
    int tag = cache[index][assoc_entry].tag;
    for(i = assoc_entry; i < cache_assoc-1 && cache[index][i+1].valid; i++)
    {
        cache[index][i].tag = cache[index][i+1].tag;
    }
    cache[index][i].tag = tag;
}

/*
 * Check if the address is in our cache.  Update our counter statistics 
 * for cache_access, cache_hit, etc.  If our configuration supports
 * associativity we may need to check through multiple entries for our
 * desired index.  In that case we will also need to call the LRU functions.
 */
int iplc_sim_trap_address(unsigned int address)
{
    // Bit-twiddles the address to get the proper tag and index bits.
    int i;
    int tag = address >> (cache_index+cache_blockoffsetbits);
    int index = (address >> cache_blockoffsetbits) & ((1 << cache_index)-1);
    
    printf("Address %x: Tag= %x, Index= %x \n", address, tag, index);
    /* For each associative set on the cache line, checks to see if the set has an address yet by
     * checking the valid bit, and if the set is valid, then it checks to see if the tag matches
     * the tag of the set. If it reaches a non valid tag, there are no more valid sets, so it
     * stops checking, as it has missed. If it reaches the end without finding a tag match, it is
     * also a miss. Otherwise, if the tag is found, it was a hit. In the case of a hit,
     * iplc_sim_LRU_update_on_hit is called and for a miss iplc_sim_LRU_replace_on_miss is called.
     */
    for(i = 0; i < cache_assoc; i++)
    {
        if(cache[index][i].valid == 0)
        {
            break;
        }
        if(cache[index][i].tag == tag)
        {
            cache_hit++;
            iplc_sim_LRU_update_on_hit(index, i);
            return 1;
        }
    }
    cache_miss++;
    iplc_sim_LRU_replace_on_miss(index, tag);
    return 0;
}

/*
 * Just output our summary statistics.
 */
void iplc_sim_finalize()
{
    /* Finish processing all instructions in the Pipeline */
    while (pipeline[FETCH].itype != NOP  ||
           pipeline[DECODE].itype != NOP ||
           pipeline[ALU].itype != NOP    ||
           pipeline[MEM].itype != NOP    ||
           pipeline[WRITEBACK].itype != NOP) {
        iplc_sim_push_pipeline_stage();
    }
    
    // Frees the dynamically allocated memory for each cache line, and then the cache itself.
    for(int i = 0; i < (1 << cache_index); i++) {
        free(cache[i]);
    }
    free(cache);

    printf(" Cache Performance \n");
    printf("\t Number of Cache Accesses is %ld \n", cache_hit+cache_miss);
    printf("\t Number of Cache Misses is %ld \n", cache_miss);
    printf("\t Number of Cache Hits is %ld \n", cache_hit);
    printf("\t Cache Miss Rate is %f \n\n", (double)cache_miss / (double)(cache_hit+cache_miss));
    printf("Pipeline Performance \n");
    printf("\t Total Cycles is %u \n", pipeline_cycles);
    printf("\t Total Instructions is %u \n", instruction_count);
    printf("\t Total Branch Instructions is %u \n", branch_count);
    printf("\t Total Correct Branch Predictions is %u \n", correct_branch_predictions);
    printf("\t CPI is %f \n\n", (double)pipeline_cycles / (double)instruction_count);
}

/************************************************************************************************/
/* Pipeline Functions ***************************************************************************/
/************************************************************************************************/

/*
 * Dump the current contents of our pipeline.
 */
void iplc_sim_dump_pipeline()
{
    int i;
    
    for (i = 0; i < MAX_STAGES; i++) {
        switch(i) {
            case FETCH:
                printf("(cyc: %u) FETCH:\t %d: 0x%x \t", pipeline_cycles, pipeline[i].itype, pipeline[i].instruction_address);
                break;
            case DECODE:
                printf("DECODE:\t %d: 0x%x \t", pipeline[i].itype, pipeline[i].instruction_address);
                break;
            case ALU:
                printf("ALU:\t %d: 0x%x \t", pipeline[i].itype, pipeline[i].instruction_address);
                break;
            case MEM:
                printf("MEM:\t %d: 0x%x \t", pipeline[i].itype, pipeline[i].instruction_address);
                break;
            case WRITEBACK:
                printf("WB:\t %d: 0x%x \n", pipeline[i].itype, pipeline[i].instruction_address);
                break;
            default:
                printf("DUMP: Bad stage!\n" );
                exit(-1);
        }
    }
}

/*
 * Check if various stages of our pipeline require stalls, forwarding, etc.
 * Then push the contents of our various pipeline stages through the pipeline.
 */
void iplc_sim_push_pipeline_stage()
{
    /* 1. Count WRITEBACK stage is "retired" -- This I'm giving you */
    if (pipeline[WRITEBACK].instruction_address) {
        instruction_count++;
        if (debug)
            printf("DEBUG: Retired Instruction at 0x%x, Type %d, at Time %u \n",
                   pipeline[WRITEBACK].instruction_address, pipeline[WRITEBACK].itype, pipeline_cycles);
    }
    
    /* 2. Check for BRANCH and correct/incorrect Branch Prediction */
    if (pipeline[DECODE].itype == BRANCH) {
        // Found a branching instruction, so increment the branch count.
        branch_count++;
        /* If the previous instruction is a nop, then branch prediction does not apply as there
         * is enough of a delay to determine whether the branch was taken or not via forwarding.
         */
        if(pipeline[FETCH].instruction_address != 0) {
            /* When predicting taken, if the address of the instruction being fetched is not
             * immediately (i.e. 4 bytes) after the address of the branching instruction in
             * decode, then the branch was taken, so the prediction was correct, so
             * correct_branch_predictions is incremented. Otherwise, increments pipeline_cycles
             * to indicate the delay of the pipeline resulting from the incorrect branch
             * prediction. For when predicting taken, the code is similar, but switched around.
             */
            if(branch_predict_taken) {
                if(pipeline[DECODE].instruction_address+4 != pipeline[FETCH].instruction_address) {
                    correct_branch_predictions++;
                    if(debug) printf("DEBUG: Branch Taken: FETCH addr = 0x%x, DECODE instr addr = 0x%x \n", pipeline[FETCH].instruction_address, pipeline[DECODE].instruction_address);
                }
                else pipeline_cycles++;
            }
            else {
                if(pipeline[DECODE].instruction_address+4 == pipeline[FETCH].instruction_address) {
                    correct_branch_predictions++;
                    if(debug) printf("DEBUG: Branch Not Taken: FETCH addr = 0x%x, DECODE instr addr = 0x%x \n", pipeline[FETCH].instruction_address, pipeline[DECODE].instruction_address);
                }
                else pipeline_cycles++;
            }
        }
    }
    
    /* 3. Check for LW delays due to use in ALU stage and if data hit/miss
     *    add delay cycles if needed.
     */
    if (pipeline[MEM].itype == LW) {
        int inserted_nop = 0;
        data_address = pipeline[MEM].stage.lw.data_address;

        /* If there is a dependency of an r-type or branching instruction in the ALU stage, which
         * is indicated by the dest_reg of the lw instruction being the same as either the reg1
         * or the reg2 of the r-type/branching instruction, then a nop must be inserted.
         */
        if(pipeline[ALU].itype == RTYPE && (pipeline[ALU].stage.rtype.reg1 == pipeline[MEM].stage.lw.dest_reg || pipeline[ALU].stage.rtype.reg2_or_constant == pipeline[MEM].stage.lw.dest_reg)) {
            inserted_nop = 1;
        }
        if(pipeline[ALU].itype == BRANCH && (pipeline[ALU].stage.branch.reg1 == pipeline[MEM].stage.lw.dest_reg || pipeline[ALU].stage.branch.reg2 == pipeline[MEM].stage.lw.dest_reg)) {
            inserted_nop = 1;
        }
        /* Checks for a hit or miss on the data address: If it hits, prints to indicate this and
         * then if a nop must be inserted, increments pipeline_cycles to indicate this. If it
         * misses, then prints to indicate this and adds 1 less than the cache miss delay (1 more
         * is added in part 5 for a total of CACHE_MISS_DELAY cycles added) to pipeline_cycles to
         * indicate the stall resultng from the miss. pipeline_cycles is not incremented by 1 even
         * if a nop must be inserted, because in the cycles it takes to deal with the cache miss,
         * the branching instruction has gone all the way through the pipeline and the next
         * instruction is known.
         */
        if(iplc_sim_trap_address(data_address)) {
            printf("DATA HIT:\t Address: 0x%x \n", data_address);
            if(inserted_nop) pipeline_cycles++;
        }
        else {
            printf("DATA MISS:\t Address: 0x%x \n", data_address);
            pipeline_cycles += CACHE_MISS_DELAY-1;
        }
    }
    
    /* 4. Check for SW mem acess and data miss .. add delay cycles if needed */
    if (pipeline[MEM].itype == SW) {
        int inserted_nop = 0;
        data_address = pipeline[MEM].stage.sw.data_address;

        /* If there is a dependency of an r-type or branching instruction in the ALU stage, which
         * is indicated by the src_reg of the sw instruction being the same as either the reg1
         * or the reg2 of the r-type/branching instruction, then a nop must be inserted.
         */
        if(pipeline[ALU].itype == RTYPE && (pipeline[ALU].stage.rtype.reg1 == pipeline[MEM].stage.sw.src_reg || pipeline[ALU].stage.rtype.reg2_or_constant == pipeline[MEM].stage.sw.src_reg)) {
            inserted_nop = 1;
        }
        if(pipeline[ALU].itype == BRANCH && (pipeline[ALU].stage.branch.reg1 == pipeline[MEM].stage.sw.src_reg || pipeline[ALU].stage.branch.reg2 == pipeline[MEM].stage.sw.src_reg)) {
            inserted_nop = 1;
        }
        /* Checks for a hit or miss on the data address: If it hits, prints to indicate this and
         * then if a nop must be inserted, increments pipeline_cycles to indicate this. If it
         * misses, then prints to indicate this and adds 1 less than the cache miss delay (1 more
         * is added in part 5 for a total of CACHE_MISS_DELAY cycles added) to pipeline_cycles to
         * indicate the stall resultng from the miss. pipeline_cycles is not incremented by 1 even
         * if a nop must be inserted, because in the cycles it takes to deal with the cache miss,
         * the branching instruction has gone all the way through the pipeline and the next
         * instruction is known.
         */
        if(iplc_sim_trap_address(data_address)) {
            printf("DATA HIT:\t Address: %x \n", data_address);
            if(inserted_nop) pipeline_cycles++;
        }
        else {
            printf("DATA MISS:\t Address: %x \n", data_address);
            pipeline_cycles += CACHE_MISS_DELAY-1;
        }
    }
    /* 5. Increment pipe_cycles 1 cycle for normal processing */
    pipeline_cycles++;
    /* 6. push stages thru MEM->WB, ALU->MEM, DECODE->ALU, FETCH->DECODE */
    pipeline[WRITEBACK] = pipeline[MEM];
    pipeline[MEM] = pipeline[ALU];
    pipeline[ALU] = pipeline[DECODE];
    pipeline[DECODE] = pipeline[FETCH];
    
    // 7. This is a give'me -- Reset the FETCH stage to NOP via bezero */
    bzero(&(pipeline[FETCH]), sizeof(pipeline_t));
}

/*
 * This function is fully implemented.  You should use this as a reference
 * for implementing the remaining instruction types.
 */
void iplc_sim_process_pipeline_rtype(char *instruction, int dest_reg, int reg1, int reg2_or_constant)
{
    // Pushes the pipeline through and then puts the parsed rtype instruction into the fetch stage
    iplc_sim_push_pipeline_stage();
    
    pipeline[FETCH].itype = RTYPE;
    pipeline[FETCH].instruction_address = instruction_address;
    
    strcpy(pipeline[FETCH].stage.rtype.instruction, instruction);
    pipeline[FETCH].stage.rtype.reg1 = reg1;
    pipeline[FETCH].stage.rtype.reg2_or_constant = reg2_or_constant;
    pipeline[FETCH].stage.rtype.dest_reg = dest_reg;
}

void iplc_sim_process_pipeline_lw(int dest_reg, int base_reg, unsigned int data_address)
{
    // Pushes the pipeline through and then puts the parsed lw instruction into the fetch stage.
    iplc_sim_push_pipeline_stage();
    pipeline[FETCH].itype = LW;
    pipeline[FETCH].instruction_address = instruction_address;

    pipeline[FETCH].stage.lw.dest_reg = dest_reg;
    pipeline[FETCH].stage.lw.base_reg = base_reg;
    pipeline[FETCH].stage.lw.data_address = data_address;
}

void iplc_sim_process_pipeline_sw(int src_reg, int base_reg, unsigned int data_address)
{
    // Pushes the pipeline through and then puts the parsed sw instruction into the fetch stage.
    iplc_sim_push_pipeline_stage();

    pipeline[FETCH].itype = SW;
    pipeline[FETCH].instruction_address = instruction_address;

    pipeline[FETCH].stage.sw.src_reg = src_reg;
    pipeline[FETCH].stage.sw.base_reg = base_reg;
    pipeline[FETCH].stage.sw.data_address = data_address;
}

void iplc_sim_process_pipeline_branch(int reg1, int reg2)
{
    // Pushes the pipeline through and then puts the parsed branch instruction into the fetch stage.
    iplc_sim_push_pipeline_stage();

    pipeline[FETCH].itype = BRANCH;
    pipeline[FETCH].instruction_address = instruction_address;

    pipeline[FETCH].stage.branch.reg1 = reg1;
    pipeline[FETCH].stage.branch.reg2 = reg2;
}

void iplc_sim_process_pipeline_jump(char *instruction)
{
    // Pushes the pipeline through and then puts the parsed jump instruction into the fetch stage.
    iplc_sim_push_pipeline_stage();

    pipeline[FETCH].itype = JUMP;
    pipeline[FETCH].instruction_address = instruction_address;

    strcpy(pipeline[FETCH].stage.jump.instruction, instruction);
}

void iplc_sim_process_pipeline_syscall()
{
    // Pushes the pipeline through and then puts the parsed syscall instruction into the fetch stage.
    iplc_sim_push_pipeline_stage();

    pipeline[FETCH].itype = SYSCALL;
    pipeline[FETCH].instruction_address = instruction_address;
}

void iplc_sim_process_pipeline_nop()
{
    // Pushes the pipeline through and then puts the parsed nop instruction into the fetch stage.
    iplc_sim_push_pipeline_stage();

    pipeline[FETCH].itype = NOP;
    pipeline[FETCH].instruction_address = instruction_address;
}

/************************************************************************************************/
/* parse Function *******************************************************************************/
/************************************************************************************************/

/*
 * Don't touch this function.  It is for parsing the instruction stream.
 */
unsigned int iplc_sim_parse_reg(char *reg_str)
{
    int i;
    // turn comma into \n
    if (reg_str[strlen(reg_str)-1] == ',')
        reg_str[strlen(reg_str)-1] = '\n';
    
    if (reg_str[0] != '$')
        return atoi(reg_str);
    else {
        // copy down over $ character than return atoi
        for (i = 0; i < strlen(reg_str); i++)
            reg_str[i] = reg_str[i+1];
        
        return atoi(reg_str);
    }
}

/*
 * Don't touch this function.  It is for parsing the instruction stream.
 */
void iplc_sim_parse_instruction(char *buffer)
{
    int instruction_hit = 0;
    int i=0, j=0;
    int src_reg=0;
    int src_reg2=0;
    int dest_reg=0;
    char str_src_reg[16];
    char str_src_reg2[16];
    char str_dest_reg[16];
    char str_constant[16];
    
    if (sscanf(buffer, "%x %s", &instruction_address, instruction ) != 2) {
        printf("Malformed instruction \n");
        exit(-1);
    }
    
    instruction_hit = iplc_sim_trap_address( instruction_address );
    
    // if a MISS, then push current instruction thru pipeline
    if (!instruction_hit) {
        // need to subtract 1, since the stage is pushed once more for actual instruction processing
        // also need to allow for a branch miss prediction during the fetch cache miss time -- by
        // counting cycles this allows for these cycles to overlap and not doubly count.
        
        printf("INST MISS:\t Address 0x%x \n", instruction_address);
        
        for (i = pipeline_cycles, j = pipeline_cycles; i < j + CACHE_MISS_DELAY - 1; i++)
            iplc_sim_push_pipeline_stage();
    }
    else
        printf("INST HIT:\t Address 0x%x \n", instruction_address);
    
    // Parse the Instruction
    
    if (strncmp( instruction, "add", 3 ) == 0 ||
        strncmp( instruction, "sll", 3 ) == 0 ||
        strncmp( instruction, "ori", 3 ) == 0) {
        if (sscanf(buffer, "%x %s %s %s %s",
                   &instruction_address,
                   instruction,
                   str_dest_reg,
                   str_src_reg,
                   str_src_reg2 ) != 5) {
            printf("Malformed RTYPE instruction (%s) at address 0x%x \n",
                   instruction, instruction_address);
            exit(-1);
        }
        
        dest_reg = iplc_sim_parse_reg(str_dest_reg);
        src_reg = iplc_sim_parse_reg(str_src_reg);
        src_reg2 = iplc_sim_parse_reg(str_src_reg2);
        
        iplc_sim_process_pipeline_rtype(instruction, dest_reg, src_reg, src_reg2);
    }
    
    else if (strncmp( instruction, "lui", 3 ) == 0) {
        if (sscanf(buffer, "%x %s %s %s",
                   &instruction_address,
                   instruction,
                   str_dest_reg,
                   str_constant ) != 4 ) {
            printf("Malformed RTYPE instruction (%s) at address 0x%x \n",
                   instruction, instruction_address );
            exit(-1);
        }
        
        dest_reg = iplc_sim_parse_reg(str_dest_reg);
        src_reg = -1;
        src_reg2 = -1;
        iplc_sim_process_pipeline_rtype(instruction, dest_reg, src_reg, src_reg2);
    }
    
    else if (strncmp( instruction, "lw", 2 ) == 0 ||
             strncmp( instruction, "sw", 2 ) == 0  ) {
        if ( sscanf( buffer, "%x %s %s %s %x",
                    &instruction_address,
                    instruction,
                    reg1,
                    offsetwithreg,
                    &data_address ) != 5) {
            printf("Bad instruction: %s at address %x \n", instruction, instruction_address);
            exit(-1);
        }
        
        if (strncmp(instruction, "lw", 2 ) == 0) {
            
            dest_reg = iplc_sim_parse_reg(reg1);
            
            // don't need to worry about base regs -- just insert -1 values
            iplc_sim_process_pipeline_lw(dest_reg, -1, data_address);
        }
        if (strncmp( instruction, "sw", 2 ) == 0) {
            src_reg = iplc_sim_parse_reg(reg1);
            
            // don't need to worry about base regs -- just insert -1 values
            iplc_sim_process_pipeline_sw( src_reg, -1, data_address);
        }
    }
    else if (strncmp( instruction, "beq", 3 ) == 0) {
        // don't need to worry about getting regs -- just insert -1 values
        iplc_sim_process_pipeline_branch(-1, -1);
    }
    else if (strncmp( instruction, "jal", 3 ) == 0 ||
             strncmp( instruction, "jr", 2 ) == 0 ||
             strncmp( instruction, "j", 1 ) == 0 ) {
        iplc_sim_process_pipeline_jump( instruction );
    }
    else if (strncmp( instruction, "jal", 3 ) == 0 ||
             strncmp( instruction, "jr", 2 ) == 0 ||
             strncmp( instruction, "j", 1 ) == 0 ) {
        /*
         * Note: no need to worry about forwarding on the jump register
         * we'll let that one go.
         */
        iplc_sim_process_pipeline_jump(instruction);
    }
    else if ( strncmp( instruction, "syscall", 7 ) == 0) {
        iplc_sim_process_pipeline_syscall( );
    }
    else if ( strncmp( instruction, "nop", 3 ) == 0) {
        iplc_sim_process_pipeline_nop( );
    }
    else {
        printf("Do not know how to process instruction: %s at address %x \n",
               instruction, instruction_address );
        exit(-1);
    }
}

/************************************************************************************************/
/* MAIN Function ********************************************************************************/
/************************************************************************************************/

int main()
{
    char trace_file_name[1024];
    FILE *trace_file = NULL;
    char buffer[80];
    int index = 10;
    int blocksize = 1;
    int assoc = 1;
    
    printf("Please enter the tracefile: ");
    scanf("%s", trace_file_name);
    
    trace_file = fopen(trace_file_name, "r");
    
    if ( trace_file == NULL ) {
        printf("fopen failed for %s file\n", trace_file_name);
        exit(-1);
    }
    
    printf("Enter Cache Size (index), Blocksize and Level of Assoc \n");
    scanf( "%d %d %d", &index, &blocksize, &assoc );
    
    printf("Enter Branch Prediction: 0 (NOT taken), 1 (TAKEN): ");
    scanf("%d", &branch_predict_taken );
    
    iplc_sim_init(index, blocksize, assoc);
    
    while (fgets(buffer, 80, trace_file) != NULL) {
        iplc_sim_parse_instruction(buffer);
        if (dump_pipeline)
            iplc_sim_dump_pipeline();
    }
    
    iplc_sim_finalize();
    return 0;
}
