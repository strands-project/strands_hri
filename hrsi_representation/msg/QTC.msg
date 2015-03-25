string uuid                 # The uuid take from the tracker
string qtc_type             # qtcb|qtcc|qtcbc
float64 quantisation_factor # The quantisation factor for 0 states
float64 smoothing_rate      # Smoothed over X seconds
float64 distance_threshold  # Distance threshold for QTC_BC
bool validated              # State chain has been validated
bool collapsed              # Similar adjacent states have been collapsed
string k                    # Agent 1
string l                    # Agent 2
string qtc_serialised       # QTC representation of type qtc_type, serialised using json
