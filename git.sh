#!/usr/bin/expect -f
spawn git add *
spawn git commit -m "commit"
spawn git push
expect "*?Username:*"
send -- "bommix\r"
expect "*?Password:*"
send -- "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"
send -- "\r"
send -- "\r"
expect eof
