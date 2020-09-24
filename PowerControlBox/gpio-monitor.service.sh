#!/bin/sh

CFG=/etc/gpio-monitor.service.cfg
test -f "$CFG" || exit 0

cat "$CFG" | while read -r pin what rest ; do
  case "$pin" in
    *"#"*|"") continue ;;
  esac
  
  case "$what" in
    on) gpio -g mode "$pin" output && gpio -g write "$pin" 1 || exit 1 ;;
    off) gpio -g mode "$pin" output && gpio -g write "$pin" 0 || exit 1 ;;
    pullup) gpio -g mode "$pin" input && gpio -g mode "$pin" up || exit 1 ;;
    pulldown) gpio -g mode "$pin" input && gpio -g mode "$pin" down || exit 1 ;;
    0|1|0,0|0,1|1,0|1,1) ;;
    *) echo 1>&2 "Unknown: $pin => $what" ; exit 1 ;;
  esac
done

pinlist="$(sed -E -n 's/^\s*([0-9]+)\s(0|1|0,0|0,1|1,0|1,1).*/\1/p' "$CFG" | sort -u)"

for pin in $pinlist ; do
(

while true ; do
  state="$(gpio -g read $pin)"
  gpio -g wfi $pin both
  newstate="$(gpio -g read $pin)"
  test "$state" = "$newstate" && continue
  sleep 1
  newstate2="$(gpio -g read $pin)"
  if grep -E -q '^\s*'$pin'\s+'${newstate}'\s+' "$CFG"; then
    sh -c "$(sed -E -n 's/^\s*'$pin'\s+'${newstate}'\s+//p' "$CFG")"
  elif grep -E -q '^\s*'$pin'\s+'${newstate},${newstate2}'\s+' "$CFG"; then
    sh -c "$(sed -E -n 's/^\s*'$pin'\s+'${newstate},${newstate2}'\s+//p' "$CFG")"
  fi
done

) &
done

wait
