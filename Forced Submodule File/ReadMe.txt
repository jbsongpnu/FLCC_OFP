This folder contains files that should be managed manually due to gitignore

1. Copy common.xml to /modules/mavlink/message_definitions/v1.0/ to overwrite existing common.xml file.
2. cubepilot.xml has conflicting msg id number. So, change those conflicting ids to unused numbers.
   (50001~50005 -> 70001~70005 in modules/mavlink/message_definitions/v1.0/cubepilot.xml)

Note: This common.xml is rebuilt on the pristine ArduPilot 4.7.1 upstream common.xml
      (do NOT reuse the 4.6.2-based file). It appends the 7 KAL HD ICD messages
      (id 50001,50002,50004,51001,51002,51003,51005) before </messages>.
