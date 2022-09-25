	.file "foc-asm.S"

	;; Consider changing this to be in the text section to reduce startup time
	;; Requires consideration of Program Space Visibility
	.section .data
inv_sqrt_3:	.2byte 0b0100100111100110
;; kp:	.2byte 0b0000110011001100 ; 0.1
;; ki:	.2byte 0 ;; 0b0000110011001100 ; 0.1
;; kd:	.2byte 0 ;; 0b0000110011001100 ; 0.1
;; v_min:	.2byte 0b1100000000000000 ; -0.5
;; v_max:	.2byte 0b0100000000000000 ; 0.5

;; vel_kp:	.2byte 0b0100000000000000 ;; 0b0000110011001100 ; 0.1
;; vel_ki:	.2byte 0		  ; 0b000011001100110 0.1
;; vel_kd:	.2byte 0		  ; 0b0000110011001100 ; 0.1
;; vel_min:	.2byte 0b1100000000000000 ; -0.5
;; vel_max:	.2byte 0b0100000000000000 ; 0.5
	
prev_err_d:	.2byte 0
prev_err_q:	.2byte 0

integral_d:	.2byte 0, 0, 0
integral_q:	.2byte 0, 0, 0

vel_prev_err:	.2byte 0
vel_integral:	.2byte 0, 0, 0

	.global _curr_velocity, _target_velocity
_curr_velocity:	.2byte 0
_target_velocity:	.2byte 0

	.global _id, _iq
_id:	.2byte 0
_iq:	.2byte 0

	.global _target_q, _target_d
_target_q:	.2byte 0
_target_d:	.2byte 0

	.global _vq, _vd
_vq:		.2byte 0
_vd:		.2byte 0
	.global _con_vq, _con_vd
_con_vq:	.2byte 0
_con_vd:	.2byte 0
	
	.global _pwm_ta, _pwm_tb, _pwm_tc, _svm_option
_pwm_ta:	.2byte 0
_pwm_tb:	.2byte 0
_pwm_tc:	.2byte 0
_svm_option:	.2byte 0

	.text
	.global _pid_update
_pid_update:
	;; [w0]=3-word array for kp, ki and kd,
	;; [w1]=2-word array for min and max outputs
	;; w2=current error, [w3]=previous error,
	;; [w4]=3-word array for integral term
	;; [w5]=4-word array of shifts (P, I, D, result)
	mov CORCON, w6		; Save CORCON
	push w6
	mov #0x00c0, w6 	; Normal saturation, signed multiply, fractional
	mov w6, CORCON
	
	;; Load integral term into accumulator B
	lac.d [w4], B	

	;; Update integral term: add ki*w3
	clr A
	mov [w0+2], w7		; ki in w7
	mov w2, w6		; current error in w6
	mac w6*w7, A		; Put ki*err in accumulator A

	mov [w5+2], w7		; Put I shift in w7
	sftac A, w7		; Shift integral contribution

	mov [w5+6], w7		; Put result shift in w7
	sftac A, w7		; Shift integral contribution by result shift
	
	add B 			; Add shifted integral contribution to integral term
	
	;; Add proportional and differential terms to accumulator A
	clr A
	mov [w0], w7		; kp in w7
	mac w6*w7, A		; Add proportional term

	mov [w5], w7		; Put P shift in w7
	sftac A, w7		; Shift P term
	
	mov [w0+4], w7		; kd in w7
	sub w6, [w3], w6	; Change in error in w6
	mac w6*w7, A

	mov [w5+6], w7		; Put result shift in w7
	sftac A, w7		; Shift P and D terms by result shift

	;; Update previous error
	mov w2, [w3]

	;; Calculate output
	add A			; Add integral term to accumulator A

	;; Apply limits
	mov [w1+2], w6
	mov [w1], w7		; Min in w7, max in w6
	sac A, w5		; Store accumulator in w5
	flim.v w6, w5, w6	; Limit output to within bounds, result in w5, excess in w6
	lac w6, A		; Put excess in accumulator A
	sub B			; Subtract excess from integral term

	;; Store integral term
	sac.d B, [w4]

	pop w6			; Restore CORCON
	mov w6, CORCON
	
	;; Return the output
	mov w5, w0

	return

	.global _velocity_pid_update
_velocity_pid_update:
	;; Load parameters
	mov #_velocity_pid_consts, w0
	mov #_pid_current_limits, w1
	mov _target_velocity, w2
	mov _curr_velocity, w3

	;; Calculate error (target - current)
	sub w2, w3, w2
	
	mov #vel_prev_err, w3
	mov #vel_integral, w4
	mov #_vel_pid_shifts, w5

	;; Call the PID function
	rcall _pid_update

	mov w0, _target_q
	mov #0, w1
	mov w1, _target_d
	
	return
	
	.global _foc_update
	;; Phase A and B currents stored in w0 and w1 respectively
	;; Sin and cos of phase angle stored in w2 and w3 respectively
_foc_update:
	push w8
	push w9

	push w0
	push w1
	push w2
	push w3

	;; Update velocity PID loop
	;; rcall _velocity_pid_update

	pop w3
	pop w2
	pop w5 			; w1 -> w5
	pop w6			; w0 -> w6
		
	;; Clarke transform - convert phase currents to alpha and beta components
	;; I_beta = I_b/sqrt(3) + I_a/(2*sqrt(3))
	;; I_b in w5, I_a in w6
	clr A 			; Clear accumulator A
	mov inv_sqrt_3, w4	; Move 1/sqrt(3) into w4
	mac w4*w5, A		; Multiply I_b by 1/sqrt(3), add to acc A
	asr w6, #1, w6		; w6 = w6/2 = I_a/2
	mac w4*w6, A		; Add I_a/(2*sqrt(3)) to acc A
	sac A, w5		; Store I_beta in w5; I_alpha is in w6
	
	;; Rotate currents to rotor reference frame
	;; I_q = I_beta * cos(theta) - I_alpha * sin(theta)
	;; I_d = I_beta * sin(theta) + I_alpha * cos(theta)
	;; sin(theta) and cos(theta) stored in w2 and w3 respectively
	clr A			; Clear accumulator A
	mov w2, w4
	mov w3, w7		; sin in w4, cos in w7
	mac w5*w7, A		; I_beta * cos
	msc w4*w6, A		; I_alpha * sin
	sac A, w8		
	clr A
	mac w4*w5, A		; I_beta * sin
	mac w6*w7, A		; I_alpha * cos
	sac A, w9		; I_q in w8, I_d in w9

	mov w8, _iq
	mov w9, _id
	
	;; Keep sin and cos safe
	push w4
	push w7
	
	;; PID loop to calculate Vd and Vq
	;; Target currents stored in _target_q and _target_d
	mov #_current_pid_consts, w0		; Constants in w0
	mov #_pid_voltage_limits, w1		; Limits in w1
	mov _target_d, w2
	sub w9, w2, w2		; w2 = I_d - _target_d
	mov #prev_err_d, w3	; Previous error ptr in w3
	mov #integral_d, w4	; Integral ptr in w4
	mov #_current_pid_shifts, w5		; No integral shift
	
	rcall _pid_update
	mov w0, w9		; Vd in w9
	
	mov #_current_pid_consts, w0		; Constants in w0
	mov #_pid_voltage_limits, w1		; Limits in w1
	mov _target_q, w2
	sub w8, w2, w2		; w2 = I_q - _target_q
	mov #prev_err_q, w3	; Previous error ptr in w3
	mov #integral_q, w4	; Integral ptr in w4
	mov #_current_pid_shifts, w5		; No integral shift
	
	rcall _pid_update
	mov w0, w6
	mov w9, w5		; Vq in w6, Vd in w5

	mov w6, _con_vq
	mov w5, _con_vd
	
	;; Comment out!
	;; mov _vq, w6
	;; mov _vd, w5
	
	;; Get sin and cos back
	pop w7
	pop w4			; sin in w4, cos in w7
	
	;; Rotate Vd and Vq into stationary reference frame
	;; V_alpha = V_d * cos(theta) - V_q * sin(theta)
	;; V_beta = V_d * sin(theta) + V_q * cos(theta)
	clr A			; Clear accumulator A
	mac w5*w7, A		; V_d * cos
	msc w4*w6, A		; V_q * sin
	sac A, w1
	clr A
	mac w4*w5, A		; V_d * sin
	mac w6*w7, A		; V_q * cos
	sac A, w5		
	mov w1, w4		; V_alpha in w4, V_beta in w5
	
	;; Space Vector Modulation determines waveforms
	;; Split into three scenarios:
	;; - First 120 degrees (V_beta > - sqrt(3) V_alpha, and V_beta > 0):
	;;     t_a = V_alpha + 1/sqrt(3) V_beta, t_b = 2/sqrt(3) V_beta
	;; - Second 120 degrees (V_beta < - sqrt(3) V_alpha, and V_beta > sqrt(3) V_alpha):
	;;     t_b = - V_alpha + 1/sqrt(3) V_beta, t_c = - V_alpha - 1/sqrt(3) V_beta
	;; - Third 120 degrees (V_beta < sqrt(3) V_alpha, and V_beta < 0):
	;;     t_a = V_alpha - 1/sqrt(3) V_beta, t_c = -2/sqrt(3) V_beta
	;; Store t_a, t_b and t_c in pwm_ta, pwm_tb, pwm_tc, and option number in svm_option
	mov inv_sqrt_3, w6
	mpy w5*w6, A		; Multiply V_beta by 1/sqrt(3)
	sac A, w6		; V_beta/sqrt(3) in w6, V_alpha in w4
	neg w4, w3		; - V_alpha in w3
	mov #0, w1
	
	cpblt w6, w3, not_1	; Rule out option 1 if V_beta/sqrt(3) < - V_alpha
	cpblt w5, w1, not_1	; Rule out option 1 if V_beta < 0

	;; Solution is in first 120 degrees
	mov #1, w8
	add w4, w6, w0		; t_a in w0 = V_alpha + 1/sqrt(3) V_beta
	add w6, w6, w1		; t_b in w1 = 2/sqrt(3) V_beta
	mov #0, w2		; t_c in w2 = 0
	goto done_svm

not_1:	cpbgt w6, w3, opt_3	; Rule out option 2 if V_beta/sqrt(3) > - V_alpha
	cpblt w6, w4, opt_3	; Rule out option 2 if V_beta/sqrt(3) < V_alpha

	;; Solution is in second 120 degrees
	mov #2, w8
	mov #0, w0		; t_a = 0
	add w3, w6, w1 		; t_b = - V_alpha + 1/sqrt(3) V_beta
	sub w3, w6, w2		; t_c = - V_alpha - 1/sqrt(3) V_beta
	goto done_svm

opt_3:
	;; Solution is in final 120 degrees
	mov #3, w8
	sub w4, w6, w0		; t_a = V_alpha - 1/sqrt(3) V_beta
	mov #0, w1		; t_b = 0
	add w6, w6, w2
	neg w2, w2		; t_c = - 2/sqrt(3) V_beta

done_svm:
	mov w0, _pwm_ta
	mov w1, _pwm_tb
	mov w2, _pwm_tc
	mov w8, _svm_option

	
	pop w9
	pop w8

	return
