Header header
uint8 in_command
int16 in_updated_needed
int32 in_angle
uint8 out_status # 0 = UNKNOWN; 1 = CONVERGED; 2 = FAILED; 3 = NOT USED;
uint8 out_angle
uint8 out_state #  0 = ALIGNMENT OFF; 1 = ALIGN_CAL_STATE_INIT; 2 = AUTO_ALIGNMENT;
                #  3 = FACTORY_ALIGNMENT; 4 = SERVICE_ALIGNMENT, 5 = RESUME_SERVICE_PT_ALIGNMENT
uint16 out_updates_completed
