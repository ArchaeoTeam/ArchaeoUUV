#!/usr/bin/expect -f
spawn git add *
spawn git commit -m "commit"
spawn git push
expect "*?//github.com':*"
send -- "bommix\r"
expect "*?bommix@github.com':*"
send -- "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"
send -- "\r"
send -- "\r"
expect eof 
