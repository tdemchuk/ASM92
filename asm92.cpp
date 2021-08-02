
/*
    ASM92 - COSC 3P92 Assembler

    ============================================================================
    Used to assemble ISA code to machine code, readable by logic circuit

    author: Tennyson Demchuk
    date:   11.30.2020

    compilation command: g++ asm.cpp -O3 -o asm92
    ============================================================================

    Input Code syntax:
        # this is a comment
        MOV $04, 3      // (0x04) = 3
        ADD $04, 5      // (0x04) = (0x04) + 5

    Notes:
        *Values are in hex
        *'#' indicates a comment
        *'$' prefix indicates a memory reference. Lack of '$' indicates
            an immediate value
        *lowercase is allowed -> converted to uppercase for mapping however

    Jumps / Branches:
     - JMP X    uncondonditional jump, X is absolute memory address
     - BR X     unconditional relative branch, X is offset from PC
     - BRZ X    conditional relative branch, X is offset from PC
     - BRN X    conditional relative branch, X is offset from PC
     - JSR X    unconditional jump to subroutine, X is absolute mem address

    Notes:
        * 'X' can either be an immediate hex value representing a memory address
            or a label used elsewhere in the program. Correct X will be calculated
            based on jump/branch instr. used if label provided
            eg.
                1 aLabel:
                2       ADD A, X
                3       ...
                ...
                24  JMP aLabel          // Translated to JMP 2
        * 'X' must be in 2's complement for relative branch instructions
            eg. Branch relative -4
                4 -> 0000 0100
                2's comp:
                          11
                    1111 1011
                  +         1
                  -----------
                    1111 1100   -->  0xFC
                
                Thus, 'BR FC' will branch to an address equal to PC - 4
        * If the outgoing carry out signal from the PSW is not fed directly into the 
            carry in (Cin) of the ALU, then modify the "ALU_CARRY_ADJUST" preprocessor
            macro below to 1 (instead of 2) and recompile this assembler. This ensures
            that the offset for a back branch (branching to a label before the 
            current instruction) is computed correctly.

    Assembler Directives:
     - Specified by '@' followed by directive name
     - Supported directives are:
        - 'base_addr' - sets the base address for program in memory, modfying all addresses in
            program accordingly. Default is 0x00.
            Usage: @base_addr=1F        // sets base address to 0x1F

    TODO : Fix label bug - labels must contain at least 1 valid hex character otherwise program will
            error. (ie. 'loop1' vs 'loop') - any label should be valid. Perhaps during label parsing
            (1st pass) only validate mnemonics before skipping to next line.
    TODO : convert to one pass compiler - write undefined labels to cache and write temp value to
            file. Once a label is found iterate through undefined label and check for matches, 
            reseeking to temp value in file and overwriting with appropriate label address. 
    TODO : Add decimal value support
*/

#include "strim.h"          // http://www.martinbroadhurst.com/how-to-trim-a-stdstring.html
#include <iostream>
#include <string>
#include <fstream>
#include <unordered_map>    
#include <set>
#include <regex>

#define ALU_CARRY_ADJUST 2

// function prototypes
void load(std::ifstream& conf);           // loads instruction mapping from 'mapping.conf' file in same directory
void parse(std::ifstream& in, std::ofstream& out, std::string& infilename, std::string& outfilename, bool write);

/*
    Instruction Map
    - Maps a 32 bit instruction code to instructions MPC address
    - For each instruction in ISA, add corresponding {instruction code, MPC address} entry 
        to instruction map
    - Instruction code calculated as follows:
        1. extract instruction mnemonic (eg. ADD)
        2. place ASCII values of the 3 chars of the mnemonic in the 3 most significant bytes
            (for 2 char mnemonic (eg. OR) leave third byte empty/0x00)

            32                                             0
            .----------------------------------------------.
            |     A     |     D     |     D     |          |
            *----------------------------------------------*
        3. calculate operand type code(s) according to the following table

            | Operand Type   | 4-bit Value |  Hex  |
            +----------------+-------------+-------+
            | No Operand     |    0000     |  0x0  |
            | Immediate      |    0001     |  0x1  |
            | Direct Address |    0010     |  0x2  |
            | -undefined-    |   > 0010    | > 0x2 |

            thus for both operands in each ISA instruction, construct
            final byte of instruction code as follows:

            8           0    
            .-----------.
            | op1 | op2 |
            *-----------*

            32                                  8          0
            .----------------------------------------------.
            |     A     |     D     |     D     | op1 |op2 |
            *----------------------------------------------*

        Eg. Instruction ADD A, X (A = direct address, X = immediate) that corresponds
        to MPC address 0x0B in Micro Store ROM.

            A = 0x41
            D = 0x44
            D = 0x44

            Operand 1 = A = Direct Address  -> operand code 1 = 0x2
            Operand 2 = X = Immediate       -> operand code 2 = 0x1

            Which gives the following instruction code:

            32                                  8          0
            .----------------------------------------------.
            |    0x41   |    0x44   |    0x44   |   0x21   |
            *----------------------------------------------*

            Or: 0x41444421

            imap entry would then look like:

                { 0x41444421, 0x0B }    // ADD A, X maps to MPC address 0x0B
*/
std::unordered_map<uint32_t, unsigned char> imap ({
                        {0x484C5400, 0x03},         // HLT [test mapping]
                        {0x4D4F5621, 0x04},         // MOV A, X [test mapping]
                        {0x41444421, 0x0B},         // ADD A, X [test mapping]
                        {0x4A4D5010, 0x50},         // JMP X [test mapping]
                        {0x42520010, 0x80}          // BR X [test mapping]
});
std::unordered_map<std::string, unsigned char> directives ({
    {"base_addr", 0x00}                             // base address of program in memory
});
bool adjustBase = false;
const std::set<std::string> jmpcodes ({     // valid jump/branch mnemonics
    {"JMP", "JSR", "BR", "BRZ", "BRN"}
});


int main(int argc, char* argv[]) {

    // local vars
    std::string infilename;                                 // assembly instruction text file
    std::string outfilename = "ram.b";                      // assembled binary file. default = out.b
    const std::string confFilename = "mapping.conf";
    
    // print header
    std::cout << "\n\
    \t      3P92 Assembler\n\
    ===================================\n\
    \tWritten By Tennyson Demchuk\n\
    \tv1.0 December 2020\n\
    ===================================\n\
    \n";

    // validate input
    if (argc < 2) {
        std::cerr << "Invalid Input. Assembly File Required:\n\
        Program Usage: ./asm code.txt [out.b]\n";
        return -1;
    }

    // display help if requested
    if (argc == 2) {
        if (std::string(argv[1]) == "help") {
            std::cout << "\n\
General Usage\n\
-------------\n\
To access help (this text): \"./asm help\"\n\n\
To execute: \"./asm CODEFILE.asm [OUTPUTFILE.b]\"\n\
Where CODEFILE.asm is the plaintext file containing ISA level instructions and OUTPUTFILE.b is the assembled binary output file that can be loaded into RAM modules in Logic Circuit. OUTPUTFILE is an optional parameter and will be named \"ram.b\" by default.\n\n\
Note: ensure the \"mapping.conf\" file is in the same directory as this executable and contains the mappings from each ISA level Mnemonic + Operand Pattern to the corresponding MPC address for each supported instruction.\n\
ie. \"ADD A, X : 4C\" in the mapping file indicates to the assembler that ADD A, X begins at MPC address 0x4C.\n\n\
Writing Code Files\n\
------------------\n\
Input Code syntax:\n\
    # this is a comment\n\
    MOV $04, 3      // (0x04) = 3\n\
    ADD $04, 5      // (0x04) = (0x04) + 5\n\
\n\
Notes:\n\
    *Values are in hex\n\
    *'#' indicates a comment\n\
    *'$' prefix indicates a memory reference. Lack of '$' indicates\n\
        an immediate value\n\
    *lowercase is allowed -> converted to uppercase for mapping however\n\
\n\
Jumps / Branches:\n\
    - JMP X    uncondonditional jump, X is absolute memory address\n\
    - BR X     unconditional relative branch, X is offset from PC\n\
    - BRZ X    conditional relative branch, X is offset from PC\n\
    - BRN X    conditional relative branch, X is offset from PC\n\
    - JSR X    unconditional jump to subroutine, X is absolute mem address\n\
\n\
Notes:\n\
    * 'X' can either be an immediate hex value representing a memory address\n\
        or a label used elsewhere in the program. Correct X will be calculated\n\
        based on jump/branch instr. used if label provided\n\
        eg.\n\
            1   aLabel:\n\
            2       ADD A, X\n\
            3       ...\n\
            ...\n\
            24  JMP aLabel          // Translated to JMP 2\n\
\n\
    * 'X' must be in 2's complement for relative branch instructions\n\
        eg. Branch relative -4\n\
            4 -> 0000 0100\n\
            2's comp:\n\
                      11\n\
                1111 1011\n\
                +       1\n\
                -----------\n\
                1111 1100   -->  0xFC\n\
 \n\
            Thus, 'BR FC' will branch to an address equal to PC - 4\n\
\n\
    * If the outgoing carry out signal from the PSW is not fed directly into the \n\
        carry in (Cin) of the ALU, then modify the \"ALU_CARRY_ADJUST\" preprocessor\n\
        macro to 1 (instead of 2) and recompile this assembler. This ensures that\n\
        the offset for a back branch (branching to a label before the current\n\
        instruction) is computed correctly.\n\
\n\
Assembler Directives:\n\
    - Specified by '@' followed by directive name\n\
    - Supported directives are:\n\
    - 'base_addr' - sets the base address for program in memory, modfying all addresses in\n\
        program accordingly. Default is 0x00.\n\
        \n\
        Usage: @base_addr=1F        // sets base address to 0x1F\n\
    \n";
            return 0;
        }
    }

    else if (argc > 3) {
        std::cerr << "Invalid Input. Too Many Arguments:\n\
        Program Usage: ./asm code.txt [out.b]\n";
        return -1;
    }

    // fetch args
    infilename = std::string(argv[1]);
    if (argc == 3) outfilename = std::string(argv[2]);

    // parse input file
    std::ifstream in(infilename);
    if (!in.is_open()) {
        std::cerr << "Error opening " << infilename << ".\n";
        return -1;
    }
    std::ofstream out(outfilename, std::ios::binary);
    if (!out.is_open()) {
        std::cerr << "Error creating " << outfilename << ".\n";
        return -1;
    }

    // load mapping config if file present
    std::ifstream conf(confFilename);
    if (conf.is_open()) {
        load(conf);
        conf.close();
    }

    // parse labels
    parse(in, out, infilename, outfilename, false);

    // parse code
    std::cout << "\nAddr.\tByte\tInstr.\n";
    in.clear();
    in.seekg(0, in.beg);
    parse(in, out, infilename, outfilename, true);

    in.close();
    out.close();

    return 0;
}

// load instruction mapping configuration
void load(std::ifstream& conf) {
    std::string line;
    int linenum = 0;
    std::regex rgx("^.*:");     // match anything before a colon on a line
    std::smatch match;
    std::string instr;
    std::string mnemonic;
    std::string map;
    uint32_t icode;
    uint32_t buffer;
    char c;
    int numops;
    unsigned char optype[2] = {0,0};
    unsigned char mpc, val;
    int i;

    while(getline(conf, line)) {
        line = trim(line);
        linenum++;
        if (line == "")     continue;
        if (line[0] == '#') continue;

        if (std::regex_search(line, match, rgx)) {
            instr = std::string(match[0]).substr(0, match[0].length()-1);
            map = line.substr(instr.length()+1, line.length() - instr.length());
            instr = trim(instr);
            map = trim(map);

            i = 0;
            mnemonic = "";
            numops = 0;
            optype[0] = 0;
            optype[1] = 0;
            while (i < instr.length()) {         // read mnemonic 
                c = instr[i++];
                if (c == ' ') break;
                mnemonic += toupper(c);
            }
            while (i < instr.length()) {         // read operands
                c = toupper(instr[i++]);
                if (c == ' ') continue;
                if (c == ',') {
                    if (numops == 0) numops++;
                    else {
                        std::cerr << "Error: Leading comma in instruction: \"" << line << "\" [line " << linenum << "]\n";
                        conf.close();           // close file
                        exit(EXIT_FAILURE);
                    }
                    continue;
                }
                if (c == 'A' || c == 'B') {     // direct mem address
                    optype[numops] = 2;
                    continue;
                }
                if (c == 'X') {
                    optype[numops] = 1;
                    continue;
                }
                std::cerr << "Error: Invalid operand type specified: '" << c << "' [line " << linenum << "]\n";
                conf.close();                   // close file
            }
            if (optype[1] != 0)         numops = 2;
            else if (optype[0] != 0)    numops = 1;

            // construct instruction code
            icode = 0;
            buffer = 0;
            for (int i=0; i < mnemonic.length(); i++) {
                if (i > 2) {
                    std::cerr << "Error: Invalid Mnemonic: \"" << mnemonic << "\" [line " << linenum << "]\n";
                    conf.close();               // close file
                    exit(EXIT_FAILURE);
                }
                buffer = mnemonic[i];           // insert mnemonic values into high order 24 bits
                buffer <<= (8 * (3-i));
                icode |= buffer;
            }
            buffer = optype[0];
            buffer <<= 4;
            buffer |= (optype[1] & 0x0F);
            icode |= buffer;

            // read MPC address
            i = 0;
            mpc = 0;
            while (i < map.length()) {
                c = toupper(map[i++]);
                if ((c >= 48 && c <= 57) || (c >= 65 && c <= 70)) {     // 0-9 or A-F
                    val = c - 48;               // assume digit
                    if (c > 64) val = c - 55;   // convert if char
                    mpc <<= 4;          // calculate operand value
                    mpc |= (val & 0x0F);
                }
                else {
                    std::cerr << "Error: Invalid MPC address: \"" << c << "\" [line " << linenum << "]. Address must be in hexadecimal.\n";
                    conf.close();               // close file
                    exit(EXIT_FAILURE);
                }
            }

            // add/update imap entry
            //std::cout << "Found instr '" << instr << "' [icode: 0x" << std::hex << icode << "] and mapped to MPC 0x" << std::hex << (int)mpc << '\n';
            imap[icode] = mpc;
        }
        else {
            std::cerr << "Error: Invalid format: \"" << line << "\" [line " << linenum << "]\n";
            conf.close();                       // close file
            exit(EXIT_FAILURE);
        }
    }
    return;
}

// parse code file
void parse(std::ifstream& in, std::ofstream& out, std::string& infilename, std::string& outfilename, bool write) {

    // local vars
    static std::unordered_map<std::string,unsigned char> lblmap;   // maps labels to addresses
    std::string line;           // current line in code file being parsed
    int linenum = 1;
    std::regex lblrgx("^.*:");  // match any string preceding a colon [simplistic and not foolproof]
    std::regex dirrgx("^.*=");  // match any string preceding an equals sign for directive pattern
    std::smatch match;          // pattern match array    
    int caddr = 0;              // address of current assembled instruction / operand
    int maddr = 0;              // memory address
    uint32_t icode;             // instruction code
    uint32_t buffer;
    std::string mnemonic;       // parsed mnemonic
    std::string lbl;            // parsed label
    int i;
    char c; 
    int numops;                 // number of operands in parsed instruction
    unsigned char ops[2] = {0,0};
    unsigned char optype[2] = {0,0};
    unsigned char val;
    unsigned char mpc;          // mpc address
    bool comment, sign;

    if (adjustBase && write) {              // adjust base by parsed offset from first pass
        caddr += directives["base_addr"];
        adjustBase = false;
    }

    while (getline(in, line)) {
        line = trim(line);

        if (line == "") {       // skip blank lines
            linenum++;
            continue;   
        }

        if (line[0] == '#') {   // skip past lines only containing a comment
            linenum++;
            continue;       
        }

        // match assembler directive
        if (line[0] == '@') {
            if (!write) {            // process only on first pass
                if (std::regex_search(line, match, dirrgx)) {
                    lbl = std::string(match[0]).substr(1, match[0].length()-2);                 // repurposing lbl and mnemonic strings temporarily
                    mnemonic = line.substr(lbl.length()+2, line.length() - lbl.length());
                    lbl = trim(lbl);
                    mnemonic = trim(mnemonic);
                    if (directives.find(lbl) != directives.end()) {
                        i = 0;
                        while (i < mnemonic.length()) {
                            c = toupper(mnemonic[i++]);
                            if (c == ' ') continue;     // skip past blank space     
                            if (c == '#') break;        // skip inline comments
                            if ((c >= 48 && c <= 57) || (c >= 65 && c <= 70)) {     // 0-9 or A-F
                                val = c - 48;               // assume digit
                                if (c > 64) val = c - 55;   // convert if char
                                mpc <<= 4;          // calculate operand value
                                mpc |= (val & 0x0F);
                            }
                            else {
                                std::cerr << "Error: Invalid hex value: \"" << mnemonic << "\" [line " << linenum << "]\n";
                                goto err;
                            }
                        }
                        directives[lbl] = mpc;      // store value under directive label
                        if (lbl == "base_addr") {
                            std::cout << "Address Offset = 0x" << std::hex << (int)mpc <<'\n';
                            caddr += mpc;       // adjust base address
                            adjustBase = true;
                        }
                    }
                    else {
                        std::cerr << "Error: Invalid assembler directive: \"" << line << "\" [line " << linenum << "]\n";
                        goto err;
                    }
                }
                else {      // all directives must match directive pattern (id=value) at the moment [if this changes, remove following error]
                    std::cerr << "Error: Invalid assembler directive assignment: \"" << line << "\" [line " << linenum << "]\n";
                    goto err;
                }
            }
            continue;
        }

        // match label
        if (std::regex_search(line, match, lblrgx)) {
            if (!write) {
                lbl = std::string(match[0]).substr(0, match[0].length()-1);
                //if (adjustBase) {
                //    caddr += directives["base_addr"];
                //    adjustBase = false;
                //}
                //std::cout << "parsed label '" << lbl << "' to address " << std::hex << caddr << "\n";
                lblmap[lbl] = caddr;    // cache mapped label and address pair in hash map
            }
            linenum++;
            continue;
        }

        // if not label, then must be instruction
        i = 0;
        mnemonic = "";
        lbl = "";
        numops = 0;
        ops[0] = 0;
        ops[1] = 0;
        optype[0] = 0;
        optype[1] = 0;
        comment = false;
        sign = false;
        while (i < line.length() && !comment) {     // read mnemonic 
            c = line[i++];
            if (c != ' ') mnemonic += toupper(c);
            else if (c == '#') comment = true;
            else break;
        }
        //std::cout << "Mnemonic: '" << mnemonic << "'\n"; 
        if (write) {           // handle jump/br instructions
            if (jmpcodes.find(mnemonic) != jmpcodes.end()) {        // mnemonic is a valid jump/branch
                numops = 1;     // all jmp/br instr. have a single immediate operand
                optype[0] = 1;

                // Since operand can be represented in code as either an immediate
                // or a label, both are computed in parallel, then a choice is made 
                // afterward
                while (i < line.length() && !comment) {     // parse rest of line
                    c = line[i++];
                    if (c == '#') {
                        comment = true;
                        continue;
                    }
                    lbl += c;                   // construct label
                    c = toupper(c);
                    val = c - 48;               // compute number val from hex
                    if (c > 64) val = c - 55;
                    ops[0] <<= 4;               // calc operand value
                    ops[0] |= (val & 0x0F);
                }
                lbl = trim(lbl);
                //std::cout << "Label = " << lbl << '\n';
                ops[0] += directives["base_addr"];      // adjust jump address by base address

                // check if valid label otherwise might be immediate value
                if (lblmap.find(lbl) != lblmap.end()) {
                    ops[0] = lblmap[lbl];                   // base adjusted address should be cached
                    if (mnemonic[0] == 'B') {               // identify if relative branch instr.
                        sign = true;
                        if (ops[0] < caddr) ops[0] = (((signed char)ops[0]) - (caddr + ALU_CARRY_ADJUST));   // back branching - must adjust for ALU carry caused by adding a 2's comp negative number (2 by default)
                        else                ops[0] = (((signed char)ops[0]) - (caddr + 1));   // forward branching - calculate relative address as signed label addr - PC [+1 because PC will be pointing to branch argument rather than branch opcode during execution]
                    }
                }
                else if (lbl.length() > 2) {        // if not label and invalid immediate (too long)
                    std::cerr << "Error: Operand is neither a valid label or immediate address: \"" << line << "\" [line " << linenum << "]\n";
                    goto err;
                }
            }
        }
        while (i < line.length() && !comment) {     // read operands
            c = toupper(line[i++]);
            if (c == ' ') continue;
            if (c == '#') {
                comment = true;
                continue;
            }
            if (c == '$') {
                optype[numops] = 2;
                continue;
            }
            if (c == ',') {
                if (numops == 0) numops++;
                else {
                    std::cerr << "Error: Leading comma in instruction: \"" << line << "\" [line " << linenum << "]\n";
                    goto err;
                }
                continue;
            }
            if ((c >= 48 && c <= 57) || (c >= 65 && c <= 70)) {     // 0-9 or A-F
                val = c - 48;               // assume digit
                if (c > 64) val = c - 55;   // convert if char
                ops[numops] <<= 4;          // calculate operand value
                ops[numops] |= (val & 0x0F);
                if (optype[numops] == 0) optype[numops] = 1;
            }
        }
        if (optype[1] != 0)         numops = 2;
        else if (optype[0] != 0)    numops = 1;

        // construct instruction code
        icode = 0;
        buffer = 0;
        for (int i=0; i < mnemonic.length(); i++) {
            if (i > 2) {
                std::cerr << "Error: Invalid Mnemonic: \"" << mnemonic << "\" [line " << linenum << "]\n";
                goto err;
            }
            buffer = mnemonic[i];           // insert mnemonic values into high order 24 bits
            buffer <<= (8 * (3-i));
            icode |= buffer;
        }
        buffer = optype[0];
        buffer <<= 4;
        buffer |= (optype[1] & 0x0F);
        icode |= buffer;

        //std::cout << "Instruction Code: 0x" << std::hex << icode << '\n';

        // map instruction code to MPC address
        if (imap.find(icode) == imap.end()) {
            //if (write) {
                std::cerr << "Error: Invalid instruction: \"" << line << "\" [line " << linenum << "]. Instruction code cannot be mapped.\n";
                std::cerr << "ICode = 0x" << std::hex << icode << '\n';
                goto err;
            //}
        }
        else {
            mpc = imap.at(icode);
            if (write) {
                std::cout << "0x" << std::hex << caddr << "\t0x" << std::hex << (int)mpc << "\t" << line << '\n';       // write to console
                out.write((char*)&mpc, 1);              // write to out file
            }
            caddr++;

            // write operands
            for (int i=0; i < numops; i++) {
                if (write)  {
                    //if (optype[i] == 2) ops[i] += directives["base_addr"];      // offset given address by base address - idk what this is for anymore & probably unnecessary but leave commented out 
                    std::cout << "0x" << std::hex << caddr << "\t0x" << std::hex << (int)ops[i] << '\n';       // write to console
                    out.write((char*)&ops[i], 1);       // write operand
                }
                caddr++;
            }
        }
        linenum++;
    }
    if (write) std::cout << '\n' << infilename << " successfully assembled to " << outfilename << " in " << std::dec << (caddr - directives["base_addr"]) << " bytes.\n";
    return;

err:
    in.close();
    out.close();                            // close out file
    std::remove(outfilename.c_str());       // delete out file
    exit(EXIT_FAILURE);
}