############################################################################
# apps/examples/hello/Make.defs
#
# Licensed to the Apache Software Foundation (ASF) under one or more
# contributor license agreements.  See the NOTICE file distributed with
# this work for additional information regarding copyright ownership.  The
# ASF licenses this file to you under the Apache License, Version 2.0 (the
# "License"); you may not use this file except in compliance with the
# License.  You may obtain a copy of the License at
#
#   http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
# WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  See the
# License for the specific language governing permissions and limitations
# under the License.
#
############################################################################

include $(APPDIR)/Make.defs

# TASK_ACTIVATION built-in application info

PROGNAME  = $(CONFIG_TASK_ACTIVATION_PROGNAME)
PRIORITY  = $(CONFIG_TASK_ACTIVATION_PRIORITY)
STACKSIZE = $(CONFIG_TASK_ACTIVATION_STACKSIZE)
MODULE    = $(CONFIG_TASK_ACTIVATION)

MAINSRC = task_activation.c

include $(APPDIR)/Application.mk
