std_msgs/Header header

string fsm_state          # IDLE / EXEC_ACTION / SAFE_HALT ...
string current_action     # Hover / MoveForward / None
int32 action_index
uint32 action_total

float32 progress          # 0.0 ~ 1.0 (best effort)

string message            # human-readable debug
