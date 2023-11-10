#!/bin/bash
i=0
for var in "$@"; do
    if [[ -f ${var} ]]; then
        cmp ${var} <(xmlindent "${@:1:${i}}" ${var} | sed 's/[[:blank:]]*$//') ||
            (xmlindent -w "${@:1:${i}}" ${var} && sed -i 's/[[:blank:]]*$//' ${var})
    else
        let "i+=1"
    fi
done

exit 0
