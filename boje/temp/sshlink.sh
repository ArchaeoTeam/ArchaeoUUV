#!/usr/bin/expect -f
# open sshlink.sh username password host "Befehl"

set username [lrange $argv 0 0] 
set password [lrange $argv 1 1]   
set ipaddr [lrange $argv 2 2] 
set scriptname [lrange $argv 3 3] 
set arg1 [lrange $argv 4 4] 
set arg2 [lrange $argv 5 5]
set arg3 [lrange $argv 6 6]
set arg4 [lrange $argv 7 7]
set arg5 [lrange $argv 8 8]
set arg6 [lrange $argv 9 9]
set timeout -1   

spawn ssh $username@$ipaddr $scriptname $arg1 $arg2 $arg3 $arg4 $arg5 $arg6
match_max 100000
expect "*?assword:*"
send -- "$password\r"
send -- "\r"
expect eof
