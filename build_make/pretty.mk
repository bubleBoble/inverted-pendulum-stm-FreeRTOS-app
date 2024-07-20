# -----------------------------------------------------------------------------
# Script for coloring make output, taken from:
# source: https://www.lunderberg.com/2015/08/25/cpp-makefile-pretty-output/
# 
# Arguments to `runt`:
#     $(1): command to run
#     $(2): prefix for output info string
# -----------------------------------------------------------------------------
COM_COLOR   = \033[1;34m
OBJ_COLOR   = \033[0;36m
OK_COLOR    = \033[0;32m
ERROR_COLOR = \033[0;31m
WARN_COLOR  = \033[0;33m
GRAY_COLOR  = \033[2;33m
NO_COLOR    = \033[m

OK_STRING    = "[OK]"
ERROR_STRING = "[ERROR]"
WARN_STRING  = "[WARNING]"
COM_STRING   = "BUILDING"

define runt
printf "%b" "$(COM_COLOR)$(COM_STRING) $(OBJ_COLOR)$(@F)$(NO_COLOR)\r"; \
eval $(1) 2> $@.log; \
RESULT=$$?; \
if [ $$RESULT -ne 0 ]; then \
    printf "%-60b%b" "$(COM_COLOR)$(2)$(OBJ_COLOR) $@" "$(ERROR_COLOR)$(ERROR_STRING)$(NO_COLOR)\n"   ; \
elif [ -s $@.log ]; then \
    printf "%-60b%b" "$(COM_COLOR)$(2)$(OBJ_COLOR) $@" "$(WARN_COLOR)$(WARN_STRING)$(NO_COLOR)\n"   ; \
else  \
    printf "%-60b%b" "$(COM_COLOR)$(2)$(OBJ_COLOR) $(@F)" "$(OK_COLOR)$(OK_STRING)$(NO_COLOR)\n"   ; \
fi; \
if [ -s $@.log ]; then \
    printf "$(GRAY_COLOR)%s$(NO_COLOR)\n" "$$(cat $@.log)"; \
fi; \
rm -f $@.log; \
exit $$RESULT
endef

