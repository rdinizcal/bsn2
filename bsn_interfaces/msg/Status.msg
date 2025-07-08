# Status.msg
string source   # Component name (e.g., "/g3t1_1", "/g4t1") 
string target   # Usually empty, defaults to "system"
string content  # Component status: "configured", "unconfigured", "activated", "deactivated"
string task     # Task being performed: "collect", "process", "transfer"