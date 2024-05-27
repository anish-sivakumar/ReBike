; math_asm.s
;
; Math routines (sqrt)
;
; Component: miscellaneous
;
; ******************************************************************************
; (c) 2017 Microchip Technology Inc. and its subsidiaries. You may use
; this software and any derivatives exclusively with Microchip products.
; 
; This software and any accompanying information is for suggestion only.
; It does not modify Microchip's standard warranty for its products.
; You agree that you are solely responsible for testing the software and
; determining its suitability.  Microchip has no obligation to modify,
; test, certify, or support the software.
; 
; THIS SOFTWARE IS SUPPLIED BY MICROCHIP "AS IS".  NO WARRANTIES,
; WHETHER EXPRESS, IMPLIED OR STATUTORY, APPLY TO THIS SOFTWARE,
; INCLUDING ANY IMPLIED WARRANTIES OF NON-INFRINGEMENT, MERCHANTABILITY,
; AND FITNESS FOR A PARTICULAR PURPOSE, OR ITS INTERACTION WITH
; MICROCHIP PRODUCTS, COMBINATION WITH ANY OTHER PRODUCTS, OR USE IN ANY
; APPLICATION.
; 
; IN NO EVENT WILL MICROCHIP BE LIABLE FOR ANY INDIRECT, SPECIAL,
; PUNITIVE, INCIDENTAL OR CONSEQUENTIAL LOSS, DAMAGE, COST OR EXPENSE OF
; ANY KIND WHATSOEVER RELATED TO THE USE OF THIS SOFTWARE, THE
; motorBench(R) DEVELOPMENT SUITE TOOL, PARAMETERS AND GENERATED CODE,
; HOWEVER CAUSED, BY END USERS, WHETHER MICROCHIP'S CUSTOMERS OR
; CUSTOMER'S CUSTOMERS, EVEN IF MICROCHIP HAS BEEN ADVISED OF THE
; POSSIBILITY OF SUCH DAMAGES OR THE DAMAGES ARE FORESEEABLE. TO THE
; FULLEST EXTENT ALLOWED BY LAW, MICROCHIP'S TOTAL LIABILITY ON ALL
; CLAIMS IN ANY WAY RELATED TO THIS SOFTWARE WILL NOT EXCEED THE AMOUNT
; OF FEES, IF ANY, THAT YOU HAVE PAID DIRECTLY TO MICROCHIP FOR THIS
; SOFTWARE.
; 
; MICROCHIP PROVIDES THIS SOFTWARE CONDITIONALLY UPON YOUR ACCEPTANCE OF
; THESE TERMS.
;
; ******************************************************************************


          .global   _Q15SQRT
          .global   Q15SQRT

_Q15SQRT:
Q15SQRT:
             mov.w w0,w7             
             clr.w w0                
             cpsgt.w w0,w7           
             nop
             nop
             nop
             mov.d w8,[w15++]        
             ff1l w7,w3            
             sub.w w3,#2,w1          
             sl w7,w1,w2             
             mov.w #0x8000,w0        
             sub.w w2,w0,w5          
             mov.w w5,w4             
             sl w5,#1,w5             
             mov.w #0x4000,w6        
             mul.ss w4,w6,w6         
             mul.ss w4,w5,w8         
             mov.w #0xf000,w0        
             mul.ss w0,w9,w2         
             add.w w2,w6,w6          
             addc.w w3,w7,w7         
             mul.ss w9,w5,w8         
             mov.w #0x800,w0         
             mul.ss w0,w9,w2         
             add.w w2,w6,w6          
             addc.w w3,w7,w7         
             mul.ss w9,w5,w8         
             mov.w #0xfb00,w0        
             mul.ss w0,w9,w2         
             add.w w2,w6,w6          
             addc.w w3,w7,w7         
             mul.ss w9,w5,w8         
             mov.w #0x380,w0         
             mul.ss w0,w9,w2         
             add.w w2,w6,w6          
             addc.w w3,w7,w7         
             mul.ss w9,w5,w8         
             mov.w #0xfd60,w0        
             mul.ss w0,w9,w2         
             add.w w2,w6,w6          
             addc.w w3,w7,w7         
             mul.ss w9,w5,w8         
             mov.w #0x210,w0         
             mul.ss w0,w9,w2         
             add.w w2,w6,w6          
             addc.w w3,w7,w7         
             mul.ss w9,w5,w8         
             mov.w #0xfe53,w0        
             mul.ss w0,w9,w2         
             add.w w2,w6,w6          
             addc.w w3,w7,w7         
             lsr w6,#15,w6           
             sl w7,#1,w0             
             ior.w w6,w0,w6          
             asr w7,#15,w7           
             mov.w #0x8000,w0        
             add.w w0,w6,w6          
             addc.w w7,#0,w7         
             lsr w1,#1,w2            
             subr.w w2,#16,w0        
             lsr w6,w2,w6            
             sl w7,w0,w0             
             ior.w w6,w0,w6          
             asr w7,w2,w7            
             btst.c w1,#0            
             bra nc, Sqrt_else       
             mov.w #0x5a82,w4        
             mul.ss w6,w4,w0         
             lsr w0,#15,w0           
             sl w1,#1,w1             
             ior.w w0,w1,w6          
Sqrt_else:   mov.w w6,w0             
             mov.d [--w15],w8        
             return 
             
             
          .end

