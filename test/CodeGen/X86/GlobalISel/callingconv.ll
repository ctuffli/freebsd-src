; NOTE: Assertions have been autogenerated by utils/update_llc_test_checks.py
; RUN: llc -mtriple=i386-linux-gnu -mattr=+sse2  -global-isel < %s -o - | FileCheck %s --check-prefix=ALL --check-prefix=X32 --check-prefix=X32_GISEL
; RUN: llc -mtriple=i386-linux-gnu -mattr=+sse2               < %s -o - | FileCheck %s --check-prefix=ALL --check-prefix=X32 --check-prefix=X32_ISEL
; RUN: llc -mtriple=x86_64-linux-gnu             -global-isel < %s -o - | FileCheck %s --check-prefix=ALL --check-prefix=X64 --check-prefix=X64_GISEL
; RUN: llc -mtriple=x86_64-linux-gnu                          < %s -o - | FileCheck %s --check-prefix=ALL --check-prefix=X64 --check-prefix=X64_ISEL

define i32 @test_ret_i32() {
; X32-LABEL: test_ret_i32:
; X32:       # BB#0:
; X32-NEXT:    movl $20, %eax
; X32-NEXT:    retl
;
; X64-LABEL: test_ret_i32:
; X64:       # BB#0:
; X64-NEXT:    movl $20, %eax
; X64-NEXT:    retq
  ret i32 20
}

define i64 @test_ret_i64() {
; X32_GISEL-LABEL: test_ret_i64:
; X32_GISEL:       # BB#0:
; X32_GISEL-NEXT:    movl $4294967295, %eax # imm = 0xFFFFFFFF
; X32_GISEL-NEXT:    movl $15, %edx
; X32_GISEL-NEXT:    retl
;
; X32_ISEL-LABEL: test_ret_i64:
; X32_ISEL:       # BB#0:
; X32_ISEL-NEXT:    movl $-1, %eax
; X32_ISEL-NEXT:    movl $15, %edx
; X32_ISEL-NEXT:    retl
;
; X64-LABEL: test_ret_i64:
; X64:       # BB#0:
; X64-NEXT:    movabsq $68719476735, %rax # imm = 0xFFFFFFFFF
; X64-NEXT:    retq
  ret i64 68719476735
}

define i8 @test_arg_i8(i8 %a) {
; X32_GISEL-LABEL: test_arg_i8:
; X32_GISEL:       # BB#0:
; X32_GISEL-NEXT:    leal 4(%esp), %eax
; X32_GISEL-NEXT:    movb (%eax), %al
; X32_GISEL-NEXT:    retl
;
; X32_ISEL-LABEL: test_arg_i8:
; X32_ISEL:       # BB#0:
; X32_ISEL-NEXT:    movb 4(%esp), %al
; X32_ISEL-NEXT:    retl
;
; X64-LABEL: test_arg_i8:
; X64:       # BB#0:
; X64-NEXT:    movl %edi, %eax
; X64-NEXT:    retq
  ret i8 %a
}

define i16 @test_arg_i16(i16 %a) {
; X32_GISEL-LABEL: test_arg_i16:
; X32_GISEL:       # BB#0:
; X32_GISEL-NEXT:    leal 4(%esp), %eax
; X32_GISEL-NEXT:    movzwl (%eax), %eax
; X32_GISEL-NEXT:    retl
;
; X32_ISEL-LABEL: test_arg_i16:
; X32_ISEL:       # BB#0:
; X32_ISEL-NEXT:    movzwl 4(%esp), %eax
; X32_ISEL-NEXT:    retl
;
; X64-LABEL: test_arg_i16:
; X64:       # BB#0:
; X64-NEXT:    movl %edi, %eax
; X64-NEXT:    retq
  ret i16 %a
}

define i32 @test_arg_i32(i32 %a) {
; X32_GISEL-LABEL: test_arg_i32:
; X32_GISEL:       # BB#0:
; X32_GISEL-NEXT:    leal 4(%esp), %eax
; X32_GISEL-NEXT:    movl (%eax), %eax
; X32_GISEL-NEXT:    retl
;
; X32_ISEL-LABEL: test_arg_i32:
; X32_ISEL:       # BB#0:
; X32_ISEL-NEXT:    movl 4(%esp), %eax
; X32_ISEL-NEXT:    retl
;
; X64-LABEL: test_arg_i32:
; X64:       # BB#0:
; X64-NEXT:    movl %edi, %eax
; X64-NEXT:    retq
  ret i32 %a
}

define i64 @test_arg_i64(i64 %a) {
; X32_GISEL-LABEL: test_arg_i64:
; X32_GISEL:       # BB#0:
; X32_GISEL-NEXT:    leal 4(%esp), %eax
; X32_GISEL-NEXT:    movl (%eax), %eax
; X32_GISEL-NEXT:    leal 8(%esp), %ecx
; X32_GISEL-NEXT:    movl (%ecx), %edx
; X32_GISEL-NEXT:    retl
;
; X32_ISEL-LABEL: test_arg_i64:
; X32_ISEL:       # BB#0:
; X32_ISEL-NEXT:    movl 4(%esp), %eax
; X32_ISEL-NEXT:    movl 8(%esp), %edx
; X32_ISEL-NEXT:    retl
;
; X64-LABEL: test_arg_i64:
; X64:       # BB#0:
; X64-NEXT:    movq %rdi, %rax
; X64-NEXT:    retq
  ret i64 %a
}

define i64 @test_i64_args_8(i64 %arg1, i64 %arg2, i64 %arg3, i64 %arg4, i64 %arg5, i64 %arg6, i64 %arg7, i64 %arg8) {
; X32_GISEL-LABEL: test_i64_args_8:
; X32_GISEL:       # BB#0:
; X32_GISEL-NEXT:    leal 60(%esp), %eax
; X32_GISEL-NEXT:    movl (%eax), %eax
; X32_GISEL-NEXT:    leal 64(%esp), %ecx
; X32_GISEL-NEXT:    movl (%ecx), %edx
; X32_GISEL-NEXT:    retl
;
; X32_ISEL-LABEL: test_i64_args_8:
; X32_ISEL:       # BB#0:
; X32_ISEL-NEXT:    movl 60(%esp), %eax
; X32_ISEL-NEXT:    movl 64(%esp), %edx
; X32_ISEL-NEXT:    retl
;
; X64_GISEL-LABEL: test_i64_args_8:
; X64_GISEL:       # BB#0:
; X64_GISEL-NEXT:    leaq 16(%rsp), %rax
; X64_GISEL-NEXT:    movq (%rax), %rax
; X64_GISEL-NEXT:    retq
;
; X64_ISEL-LABEL: test_i64_args_8:
; X64_ISEL:       # BB#0:
; X64_ISEL-NEXT:    movq 16(%rsp), %rax
; X64_ISEL-NEXT:    retq

  ret i64 %arg8
}

define <4 x i32> @test_v4i32_args(<4 x i32> %arg1, <4 x i32> %arg2) {
; X32-LABEL: test_v4i32_args:
; X32:       # BB#0:
; X32-NEXT:    movaps %xmm1, %xmm0
; X32-NEXT:    retl
;
; X64-LABEL: test_v4i32_args:
; X64:       # BB#0:
; X64-NEXT:    movaps %xmm1, %xmm0
; X64-NEXT:    retq
  ret <4 x i32> %arg2
}

define <8 x i32> @test_v8i32_args(<8 x i32> %arg1) {
; X32-LABEL: test_v8i32_args:
; X32:       # BB#0:
; X32-NEXT:    retl
;
; X64-LABEL: test_v8i32_args:
; X64:       # BB#0:
; X64-NEXT:    retq

  ret <8 x i32> %arg1
}
