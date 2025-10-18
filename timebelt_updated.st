PROGRAM timebelt
  VAR
    btn AT %QX0.0 : BOOL;
    motor AT %QX0.1 : BOOL;
    pulse AT %QX0.2 : BOOL;
    reset AT %QX0.3 : BOOL;
    wg_sensor AT %MW0 : INT;
    total_count AT %MW1 : INT;
    big_box AT %MW2 : INT;
    med_box AT %MW3 : INT;
    small_box AT %MW4 : INT;
  END_VAR
  VAR
    CTU0 : CTU;
    CTU1 : CTU;
    CTU2 : CTU;
    preset_value : INT := 100;
    TON0 : TON;
    TON1 : TON;
    TON2 : TON;
    big_box_stop : BOOL;
    med_box_stop : BOOL;
    small_box_stop : BOOL;
    _TMP_ADD34_OUT : INT;
    _TMP_GT4_OUT : BOOL;
    R_TRIG1 : R_TRIG;
    _TMP_GT9_OUT : BOOL;
    _TMP_LT17_OUT : BOOL;
    _TMP_AND18_OUT : BOOL;
    R_TRIG2 : R_TRIG;
    _TMP_LT25_OUT : BOOL;
    _TMP_GT6_OUT : BOOL;
    _TMP_AND3_OUT : BOOL;
    R_TRIG3 : R_TRIG;
    _TMP_OR60_OUT : BOOL;
    _TMP_NOT63_OUT : BOOL;
    _TMP_AND66_OUT : BOOL;
  END_VAR

  _TMP_ADD34_OUT := ADD(big_box, med_box, small_box);
  total_count := _TMP_ADD34_OUT;
  _TMP_GT4_OUT := GT(wg_sensor, 10);
  R_TRIG1(CLK := pulse AND _TMP_GT4_OUT);
  CTU0(CU := R_TRIG1.Q, R := reset, PV := preset_value);
  big_box := CTU0.CV;
  _TMP_GT9_OUT := GT(wg_sensor, 5);
  _TMP_LT17_OUT := LT(wg_sensor, 10);
  _TMP_AND18_OUT := AND(_TMP_GT9_OUT, _TMP_LT17_OUT);
  R_TRIG2(CLK := pulse AND _TMP_AND18_OUT);
  CTU1(CU := R_TRIG2.Q, R := reset, PV := preset_value);
  med_box := CTU1.CV;
  _TMP_LT25_OUT := LT(wg_sensor, 5);
  _TMP_GT6_OUT := GT(wg_sensor, 0);
  _TMP_AND3_OUT := AND(_TMP_LT25_OUT, _TMP_GT6_OUT);
  R_TRIG3(CLK := pulse AND _TMP_AND3_OUT);
  CTU2(CU := R_TRIG3.Q, R := reset, PV := preset_value);
  small_box := CTU2.CV;
  TON1(IN := _TMP_AND18_OUT, PT := T#3.25s);
  med_box_stop := TON1.Q;
  TON2(IN := _TMP_AND3_OUT, PT := T#5s);
  small_box_stop := TON2.Q;
  TON0(IN := _TMP_GT4_OUT, PT := T#1.5s);
  big_box_stop := TON0.Q;
  _TMP_OR60_OUT := OR(big_box_stop, med_box_stop, small_box_stop);
  _TMP_NOT63_OUT := NOT(_TMP_OR60_OUT);
  _TMP_AND66_OUT := AND(btn, _TMP_NOT63_OUT);
  motor := _TMP_AND66_OUT;
END_PROGRAM


CONFIGURATION Config0

  RESOURCE Res0 ON PLC
    TASK task0(INTERVAL := T#20ms,PRIORITY := 0);
    PROGRAM instance0 WITH task0 : timebelt;
  END_RESOURCE
END_CONFIGURATION

(*DBG:char md5[] = "f08cb03a1d368d834a3beaef3357ad20";*)
(*DBG:/**)
(*DBG: * This file is part of OpenPLC Runtime*)
(*DBG: **)
(*DBG: * Copyright (C) 2023 Autonomy, GP Orcullo*)
(*DBG: * Based on the work by GP Orcullo on Beremiz for uC*)
(*DBG: **)
(*DBG: * This program is free software; you can redistribute it and/or*)
(*DBG: * modify it under the terms of the GNU General Public License*)
(*DBG: * as published by the Free Software Foundation; either version 2*)
(*DBG: * of the License, or (at your option) any later version.*)
(*DBG: **)
(*DBG: * This program is distributed in the hope that it will be useful,*)
(*DBG: * but WITHOUT ANY WARRANTY; without even the implied warranty of*)
(*DBG: * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the*)
(*DBG: * GNU General Public License for more details.*)
(*DBG: **)
(*DBG: * You should have received a copy of the GNU General Public License*)
(*DBG: * along with this program; If not, see <http://www.gnu.org/licenses/>.*)
(*DBG: **)
(*DBG: */*)
(*DBG:*)
(*DBG:#include <stdbool.h>*)
(*DBG:*)
(*DBG:#include "iec_types_all.h"*)
(*DBG:#include "POUS.h"*)
(*DBG:*)
(*DBG:#define SAME_ENDIANNESS      0*)
(*DBG:#define REVERSE_ENDIANNESS   1*)
(*DBG:*)
(*DBG:uint8_t endianness;*)
(*DBG:*)
(*DBG:*)
(*DBG:extern TIMEBELT RES0__INSTANCE0;*)
(*DBG:*)
(*DBG:static const struct {*)
(*DBG:    void *ptr;*)
(*DBG:    __IEC_types_enum type;*)
(*DBG:} debug_vars[] = {*)
(*DBG:    {&(RES0__INSTANCE0.BTN), BOOL_O_ENUM},*)
(*DBG:    {&(RES0__INSTANCE0.MOTOR), BOOL_O_ENUM},*)
(*DBG:    {&(RES0__INSTANCE0.PULSE), BOOL_O_ENUM},*)
(*DBG:    {&(RES0__INSTANCE0.RESET), BOOL_O_ENUM},*)
(*DBG:    {&(RES0__INSTANCE0.WG_SENSOR), INT_O_ENUM},*)
(*DBG:    {&(RES0__INSTANCE0.TOTAL_COUNT), INT_O_ENUM},*)
(*DBG:    {&(RES0__INSTANCE0.BIG_BOX), INT_O_ENUM},*)
(*DBG:    {&(RES0__INSTANCE0.MED_BOX), INT_O_ENUM},*)
(*DBG:    {&(RES0__INSTANCE0.SMALL_BOX), INT_O_ENUM},*)
(*DBG:    {&(RES0__INSTANCE0.CTU0.EN), BOOL_ENUM},*)
(*DBG:    {&(RES0__INSTANCE0.CTU0.ENO), BOOL_ENUM},*)
(*DBG:    {&(RES0__INSTANCE0.CTU0.CU), BOOL_ENUM},*)
(*DBG:    {&(RES0__INSTANCE0.CTU0.R), BOOL_ENUM},*)
(*DBG:    {&(RES0__INSTANCE0.CTU0.PV), INT_ENUM},*)
(*DBG:    {&(RES0__INSTANCE0.CTU0.Q), BOOL_ENUM},*)
(*DBG:    {&(RES0__INSTANCE0.CTU0.CV), INT_ENUM},*)
(*DBG:    {&(RES0__INSTANCE0.CTU0.CU_T.EN), BOOL_ENUM},*)
(*DBG:    {&(RES0__INSTANCE0.CTU0.CU_T.ENO), BOOL_ENUM},*)
(*DBG:    {&(RES0__INSTANCE0.CTU0.CU_T.CLK), BOOL_ENUM},*)
(*DBG:    {&(RES0__INSTANCE0.CTU0.CU_T.Q), BOOL_ENUM},*)
(*DBG:    {&(RES0__INSTANCE0.CTU0.CU_T.M), BOOL_ENUM},*)
(*DBG:    {&(RES0__INSTANCE0.CTU1.EN), BOOL_ENUM},*)
(*DBG:    {&(RES0__INSTANCE0.CTU1.ENO), BOOL_ENUM},*)
(*DBG:    {&(RES0__INSTANCE0.CTU1.CU), BOOL_ENUM},*)
(*DBG:    {&(RES0__INSTANCE0.CTU1.R), BOOL_ENUM},*)
(*DBG:    {&(RES0__INSTANCE0.CTU1.PV), INT_ENUM},*)
(*DBG:    {&(RES0__INSTANCE0.CTU1.Q), BOOL_ENUM},*)
(*DBG:    {&(RES0__INSTANCE0.CTU1.CV), INT_ENUM},*)
(*DBG:    {&(RES0__INSTANCE0.CTU1.CU_T.EN), BOOL_ENUM},*)
(*DBG:    {&(RES0__INSTANCE0.CTU1.CU_T.ENO), BOOL_ENUM},*)
(*DBG:    {&(RES0__INSTANCE0.CTU1.CU_T.CLK), BOOL_ENUM},*)
(*DBG:    {&(RES0__INSTANCE0.CTU1.CU_T.Q), BOOL_ENUM},*)
(*DBG:    {&(RES0__INSTANCE0.CTU1.CU_T.M), BOOL_ENUM},*)
(*DBG:    {&(RES0__INSTANCE0.CTU2.EN), BOOL_ENUM},*)
(*DBG:    {&(RES0__INSTANCE0.CTU2.ENO), BOOL_ENUM},*)
(*DBG:    {&(RES0__INSTANCE0.CTU2.CU), BOOL_ENUM},*)
(*DBG:    {&(RES0__INSTANCE0.CTU2.R), BOOL_ENUM},*)
(*DBG:    {&(RES0__INSTANCE0.CTU2.PV), INT_ENUM},*)
(*DBG:    {&(RES0__INSTANCE0.CTU2.Q), BOOL_ENUM},*)
(*DBG:    {&(RES0__INSTANCE0.CTU2.CV), INT_ENUM},*)
(*DBG:    {&(RES0__INSTANCE0.CTU2.CU_T.EN), BOOL_ENUM},*)
(*DBG:    {&(RES0__INSTANCE0.CTU2.CU_T.ENO), BOOL_ENUM},*)
(*DBG:    {&(RES0__INSTANCE0.CTU2.CU_T.CLK), BOOL_ENUM},*)
(*DBG:    {&(RES0__INSTANCE0.CTU2.CU_T.Q), BOOL_ENUM},*)
(*DBG:    {&(RES0__INSTANCE0.CTU2.CU_T.M), BOOL_ENUM},*)
(*DBG:    {&(RES0__INSTANCE0.PRESET_VALUE), INT_ENUM},*)
(*DBG:    {&(RES0__INSTANCE0.TON0.EN), BOOL_ENUM},*)
(*DBG:    {&(RES0__INSTANCE0.TON0.ENO), BOOL_ENUM},*)
(*DBG:    {&(RES0__INSTANCE0.TON0.IN), BOOL_ENUM},*)
(*DBG:    {&(RES0__INSTANCE0.TON0.PT), TIME_ENUM},*)
(*DBG:    {&(RES0__INSTANCE0.TON0.Q), BOOL_ENUM},*)
(*DBG:    {&(RES0__INSTANCE0.TON0.ET), TIME_ENUM},*)
(*DBG:    {&(RES0__INSTANCE0.TON0.STATE), SINT_ENUM},*)
(*DBG:    {&(RES0__INSTANCE0.TON0.PREV_IN), BOOL_ENUM},*)
(*DBG:    {&(RES0__INSTANCE0.TON0.CURRENT_TIME), TIME_ENUM},*)
(*DBG:    {&(RES0__INSTANCE0.TON0.START_TIME), TIME_ENUM},*)
(*DBG:    {&(RES0__INSTANCE0.TON1.EN), BOOL_ENUM},*)
(*DBG:    {&(RES0__INSTANCE0.TON1.ENO), BOOL_ENUM},*)
(*DBG:    {&(RES0__INSTANCE0.TON1.IN), BOOL_ENUM},*)
(*DBG:    {&(RES0__INSTANCE0.TON1.PT), TIME_ENUM},*)
(*DBG:    {&(RES0__INSTANCE0.TON1.Q), BOOL_ENUM},*)
(*DBG:    {&(RES0__INSTANCE0.TON1.ET), TIME_ENUM},*)
(*DBG:    {&(RES0__INSTANCE0.TON1.STATE), SINT_ENUM},*)
(*DBG:    {&(RES0__INSTANCE0.TON1.PREV_IN), BOOL_ENUM},*)
(*DBG:    {&(RES0__INSTANCE0.TON1.CURRENT_TIME), TIME_ENUM},*)
(*DBG:    {&(RES0__INSTANCE0.TON1.START_TIME), TIME_ENUM},*)
(*DBG:    {&(RES0__INSTANCE0.TON2.EN), BOOL_ENUM},*)
(*DBG:    {&(RES0__INSTANCE0.TON2.ENO), BOOL_ENUM},*)
(*DBG:    {&(RES0__INSTANCE0.TON2.IN), BOOL_ENUM},*)
(*DBG:    {&(RES0__INSTANCE0.TON2.PT), TIME_ENUM},*)
(*DBG:    {&(RES0__INSTANCE0.TON2.Q), BOOL_ENUM},*)
(*DBG:    {&(RES0__INSTANCE0.TON2.ET), TIME_ENUM},*)
(*DBG:    {&(RES0__INSTANCE0.TON2.STATE), SINT_ENUM},*)
(*DBG:    {&(RES0__INSTANCE0.TON2.PREV_IN), BOOL_ENUM},*)
(*DBG:    {&(RES0__INSTANCE0.TON2.CURRENT_TIME), TIME_ENUM},*)
(*DBG:    {&(RES0__INSTANCE0.TON2.START_TIME), TIME_ENUM},*)
(*DBG:    {&(RES0__INSTANCE0.BIG_BOX_STOP), BOOL_ENUM},*)
(*DBG:    {&(RES0__INSTANCE0.MED_BOX_STOP), BOOL_ENUM},*)
(*DBG:    {&(RES0__INSTANCE0.SMALL_BOX_STOP), BOOL_ENUM},*)
(*DBG:    {&(RES0__INSTANCE0._TMP_ADD34_OUT), INT_ENUM},*)
(*DBG:    {&(RES0__INSTANCE0._TMP_GT4_OUT), BOOL_ENUM},*)
(*DBG:    {&(RES0__INSTANCE0.R_TRIG1.EN), BOOL_ENUM},*)
(*DBG:    {&(RES0__INSTANCE0.R_TRIG1.ENO), BOOL_ENUM},*)
(*DBG:    {&(RES0__INSTANCE0.R_TRIG1.CLK), BOOL_ENUM},*)
(*DBG:    {&(RES0__INSTANCE0.R_TRIG1.Q), BOOL_ENUM},*)
(*DBG:    {&(RES0__INSTANCE0.R_TRIG1.M), BOOL_ENUM},*)
(*DBG:    {&(RES0__INSTANCE0._TMP_GT9_OUT), BOOL_ENUM},*)
(*DBG:    {&(RES0__INSTANCE0._TMP_LT17_OUT), BOOL_ENUM},*)
(*DBG:    {&(RES0__INSTANCE0._TMP_AND18_OUT), BOOL_ENUM},*)
(*DBG:    {&(RES0__INSTANCE0.R_TRIG2.EN), BOOL_ENUM},*)
(*DBG:    {&(RES0__INSTANCE0.R_TRIG2.ENO), BOOL_ENUM},*)
(*DBG:    {&(RES0__INSTANCE0.R_TRIG2.CLK), BOOL_ENUM},*)
(*DBG:    {&(RES0__INSTANCE0.R_TRIG2.Q), BOOL_ENUM},*)
(*DBG:    {&(RES0__INSTANCE0.R_TRIG2.M), BOOL_ENUM},*)
(*DBG:    {&(RES0__INSTANCE0._TMP_LT25_OUT), BOOL_ENUM},*)
(*DBG:    {&(RES0__INSTANCE0._TMP_GT6_OUT), BOOL_ENUM},*)
(*DBG:    {&(RES0__INSTANCE0._TMP_AND3_OUT), BOOL_ENUM},*)
(*DBG:    {&(RES0__INSTANCE0.R_TRIG3.EN), BOOL_ENUM},*)
(*DBG:    {&(RES0__INSTANCE0.R_TRIG3.ENO), BOOL_ENUM},*)
(*DBG:    {&(RES0__INSTANCE0.R_TRIG3.CLK), BOOL_ENUM},*)
(*DBG:    {&(RES0__INSTANCE0.R_TRIG3.Q), BOOL_ENUM},*)
(*DBG:    {&(RES0__INSTANCE0.R_TRIG3.M), BOOL_ENUM},*)
(*DBG:    {&(RES0__INSTANCE0._TMP_OR60_OUT), BOOL_ENUM},*)
(*DBG:    {&(RES0__INSTANCE0._TMP_NOT63_OUT), BOOL_ENUM},*)
(*DBG:    {&(RES0__INSTANCE0._TMP_AND66_OUT), BOOL_ENUM},*)
(*DBG:};*)
(*DBG:*)
(*DBG:#define VAR_COUNT               105*)
(*DBG:*)
(*DBG:uint16_t get_var_count(void)*)
(*DBG:{*)
(*DBG:    return VAR_COUNT;*)
(*DBG:}*)
(*DBG:*)
(*DBG:size_t get_var_size(size_t idx)*)
(*DBG:{*)
(*DBG:    if (idx >= VAR_COUNT)*)
(*DBG:    {*)
(*DBG:        return 0;*)
(*DBG:    }*)
(*DBG:    switch (debug_vars[idx].type) {*)
(*DBG:    case TIME_ENUM:*)
(*DBG:        return sizeof(TIME);*)
(*DBG:    case SINT_ENUM:*)
(*DBG:        return sizeof(SINT);*)
(*DBG:    case INT_ENUM:*)
(*DBG:    case INT_O_ENUM:*)
(*DBG:        return sizeof(INT);*)
(*DBG:    case BOOL_ENUM:*)
(*DBG:    case BOOL_O_ENUM:*)
(*DBG:        return sizeof(BOOL);*)
(*DBG:    default:*)
(*DBG:        return 0;*)
(*DBG:    }*)
(*DBG:}*)
(*DBG:*)
(*DBG:void *get_var_addr(size_t idx)*)
(*DBG:{*)
(*DBG:    void *ptr = debug_vars[idx].ptr;*)
(*DBG:*)
(*DBG:    switch (debug_vars[idx].type) {*)
(*DBG:    case TIME_ENUM:*)
(*DBG:        return (void *)&((__IEC_TIME_t *) ptr)->value;*)
(*DBG:    case SINT_ENUM:*)
(*DBG:        return (void *)&((__IEC_SINT_t *) ptr)->value;*)
(*DBG:    case INT_ENUM:*)
(*DBG:        return (void *)&((__IEC_INT_t *) ptr)->value;*)
(*DBG:    case INT_O_ENUM:*)
(*DBG:        return (void *)((((__IEC_INT_p *) ptr)->flags & __IEC_FORCE_FLAG) *)
(*DBG:                        ? &(((__IEC_INT_p *) ptr)->fvalue) *)
(*DBG:                        : ((__IEC_INT_p *) ptr)->value);*)
(*DBG:    case BOOL_ENUM:*)
(*DBG:        return (void *)&((__IEC_BOOL_t *) ptr)->value;*)
(*DBG:    case BOOL_O_ENUM:*)
(*DBG:        return (void *)((((__IEC_BOOL_p *) ptr)->flags & __IEC_FORCE_FLAG) *)
(*DBG:                        ? &(((__IEC_BOOL_p *) ptr)->fvalue) *)
(*DBG:                        : ((__IEC_BOOL_p *) ptr)->value);*)
(*DBG:    default:*)
(*DBG:        return 0;*)
(*DBG:    }*)
(*DBG:}*)
(*DBG:*)
(*DBG:void force_var(size_t idx, bool forced, void *val)*)
(*DBG:{*)
(*DBG:    void *ptr = debug_vars[idx].ptr;*)
(*DBG:*)
(*DBG:    if (forced) {*)
(*DBG:        size_t var_size = get_var_size(idx);*)
(*DBG:        switch (debug_vars[idx].type) {*)
(*DBG:        case TIME_ENUM: {*)
(*DBG:            memcpy(&((__IEC_TIME_t *) ptr)->value, val, var_size);*)
(*DBG:            //((__IEC_TIME_t *) ptr)->value = *((TIME *) val);*)
(*DBG:            ((__IEC_TIME_t *) ptr)->flags |= __IEC_FORCE_FLAG;*)
(*DBG:            break;*)
(*DBG:        }*)
(*DBG:        case SINT_ENUM: {*)
(*DBG:            memcpy(&((__IEC_SINT_t *) ptr)->value, val, var_size);*)
(*DBG:            //((__IEC_SINT_t *) ptr)->value = *((SINT *) val);*)
(*DBG:            ((__IEC_SINT_t *) ptr)->flags |= __IEC_FORCE_FLAG;*)
(*DBG:            break;*)
(*DBG:        }*)
(*DBG:        case INT_ENUM: {*)
(*DBG:            memcpy(&((__IEC_INT_t *) ptr)->value, val, var_size);*)
(*DBG:            //((__IEC_INT_t *) ptr)->value = *((INT *) val);*)
(*DBG:            ((__IEC_INT_t *) ptr)->flags |= __IEC_FORCE_FLAG;*)
(*DBG:            break;*)
(*DBG:        }*)
(*DBG:        case INT_O_ENUM: {*)
(*DBG:            memcpy((((__IEC_INT_p *) ptr)->value), val, var_size);*)
(*DBG:            //*(((__IEC_INT_p *) ptr)->value) = *((INT *) val);*)
(*DBG:            ((__IEC_INT_p *) ptr)->flags |= __IEC_FORCE_FLAG;*)
(*DBG:            break;*)
(*DBG:        }*)
(*DBG:        case BOOL_ENUM: {*)
(*DBG:            memcpy(&((__IEC_BOOL_t *) ptr)->value, val, var_size);*)
(*DBG:            //((__IEC_BOOL_t *) ptr)->value = *((BOOL *) val);*)
(*DBG:            ((__IEC_BOOL_t *) ptr)->flags |= __IEC_FORCE_FLAG;*)
(*DBG:            break;*)
(*DBG:        }*)
(*DBG:        case BOOL_O_ENUM: {*)
(*DBG:            memcpy((((__IEC_BOOL_p *) ptr)->value), val, var_size);*)
(*DBG:            //*(((__IEC_BOOL_p *) ptr)->value) = *((BOOL *) val);*)
(*DBG:            ((__IEC_BOOL_p *) ptr)->flags |= __IEC_FORCE_FLAG;*)
(*DBG:            break;*)
(*DBG:        }*)
(*DBG:        default:*)
(*DBG:            break;*)
(*DBG:        }*)
(*DBG:    } else {*)
(*DBG:        switch (debug_vars[idx].type) {*)
(*DBG:        case TIME_ENUM:*)
(*DBG:            ((__IEC_TIME_t *) ptr)->flags &= ~__IEC_FORCE_FLAG;*)
(*DBG:            break;*)
(*DBG:        case SINT_ENUM:*)
(*DBG:            ((__IEC_SINT_t *) ptr)->flags &= ~__IEC_FORCE_FLAG;*)
(*DBG:            break;*)
(*DBG:        case INT_ENUM:*)
(*DBG:            ((__IEC_INT_t *) ptr)->flags &= ~__IEC_FORCE_FLAG;*)
(*DBG:            break;*)
(*DBG:        case INT_O_ENUM:*)
(*DBG:            ((__IEC_INT_p *) ptr)->flags &= ~__IEC_FORCE_FLAG;*)
(*DBG:            break;*)
(*DBG:        case BOOL_ENUM:*)
(*DBG:            ((__IEC_BOOL_t *) ptr)->flags &= ~__IEC_FORCE_FLAG;*)
(*DBG:            break;*)
(*DBG:        case BOOL_O_ENUM:*)
(*DBG:            ((__IEC_BOOL_p *) ptr)->flags &= ~__IEC_FORCE_FLAG;*)
(*DBG:            break;*)
(*DBG:        default:*)
(*DBG:            break;*)
(*DBG:        }*)
(*DBG:    }*)
(*DBG:}*)
(*DBG:*)
(*DBG:void swap_bytes(void *ptr, size_t size) *)
(*DBG:{*)
(*DBG:    uint8_t *bytePtr = (uint8_t *)ptr;*)
(*DBG:    size_t i;*)
(*DBG:    for (i = 0; i < size / 2; ++i) *)
(*DBG:    {*)
(*DBG:        uint8_t temp = bytePtr[i];*)
(*DBG:        bytePtr[i] = bytePtr[size - 1 - i];*)
(*DBG:        bytePtr[size - 1 - i] = temp;*)
(*DBG:    }*)
(*DBG:}*)
(*DBG:*)
(*DBG:void trace_reset(void)*)
(*DBG:{*)
(*DBG:    for (size_t i=0; i < VAR_COUNT; i++) *)
(*DBG:    {*)
(*DBG:        force_var(i, false, 0);*)
(*DBG:    }*)
(*DBG:}*)
(*DBG:*)
(*DBG:void set_trace(size_t idx, bool forced, void *val)*)
(*DBG:{*)
(*DBG:    if (idx >= 0 && idx < VAR_COUNT) *)
(*DBG:    {*)
(*DBG:        if (endianness == REVERSE_ENDIANNESS)*)
(*DBG:        {*)
(*DBG:            // Aaaaarghhhh... Stupid AVR is Big Endian.*)
(*DBG:            swap_bytes(val, get_var_size(idx));*)
(*DBG:        }*)
(*DBG:*)
(*DBG:        force_var(idx, forced, val);*)
(*DBG:    }*)
(*DBG:}*)
(*DBG:*)
(*DBG:void set_endianness(uint8_t value)*)
(*DBG:{*)
(*DBG:    if (value == SAME_ENDIANNESS || value == REVERSE_ENDIANNESS)*)
(*DBG:    {*)
(*DBG:        endianness = value;*)
(*DBG:    }*)
(*DBG:}*)
(*DBG:*)