# Sent by an agent at a few hertz while it runs, on the executor's ~/heartbeat topic. A mission
# started with a heartbeat_timeout_s is stopped when these stop arriving.

builtin_interfaces/Time stamp
# Who is alive, such as "nervros 41234": the executor's deadman matches it against a mission's
# or a lend's heartbeat_client, and an empty heartbeat_client there accepts any client's beat.
string client
