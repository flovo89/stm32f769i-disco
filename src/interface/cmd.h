#pragma once

#include "foc/foc.h"

/* Shared FOC context — set by main before starting threads */
extern foc_ctx_t g_foc;

/* Start the UDP command server (port 5000). Never returns. */
void udp_server_run(void);
