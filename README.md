# Installation

- From https://artifactory.kinovaapps.com/ui/native/generic-public/kortex/API/2.6.0/ install the .whl link (should be first file) and then install it using “pip install kortex_api-2.6.0.post3-py3-none-any.whl”
- Run “pip install protobuf==3.20.0”
  - It will show an error message but it’s fine probably
- You should be able to run any of the control_{control method}.py files such as control_kb.py
- `pip install sshkeyboard`

# Running
- Connect ethernet cable
- Go to ethernet settings -> IP assignment -> Manually set IPv4 to 192.168.2.11 and Subnet mask to 255.255.255.0
- Test out if it works by going to the robots ip address (Blue is 192.168.2.9, Green is 192.168.2.10)
  - If the website loads at all, you're good
- Run control_kb.py 

# Resources
- Notes: https://docs.google.com/document/d/1zoFW4ZuOTBR4lSNPiNPy8yPemmfg1WP5qOGI7yzQbJ0/edit?tab=t.0
- AIMS Setup Guide: https://docs.google.com/document/d/1Pn7DSofQ46O9p9HMk7hkZw4vB3vNeKJx8RcrybjuTgo/edit?tab=t.0
- AIMS API Guide: https://docs.google.com/document/d/1slQGlaatKXmACnxqd-COBkTYuNXRv5iNnrE0vGLQK2A/edit?tab=t.0