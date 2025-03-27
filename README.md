# 16-Bit-MIPS-Single-Cycle-Processor
The Instruction Format and Instruction Set Architecture for the 16-bit single-cycle MIPS are as follows:
| Name      | Fields                                  | Comments                               |
|----------|----------------------------------------|----------------------------------------|
| Field size | 3 bits | 3 bits | 3 bits | 3 bits | 4 bits | All MIPS-L instructions 16 bits |
| R-format  | op | rs | rt | rd | funct | Arithmetic instruction format |
| I-format  | op | rs | rt | Address/immediate | Transfer, branch, immediate format |
| J-format  | op | target address | Jump instruction format |

