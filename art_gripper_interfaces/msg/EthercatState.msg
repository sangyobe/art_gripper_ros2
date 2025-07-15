uint32 slaves_responding        # Sum of responding slaves on all Ethernet devices.
uint8 al_states                 # Application-layer states of all slaves.
                                #  The states are coded in the lower 4 bits.
                                #  If a bit is set, it means that at least one
                                #  slave in the network is in the corresponding
                                #  state:
                                #  - Bit 0: \a INIT
                                #  - Bit 1: \a PREOP
                                #  - Bit 2: \a SAFEOP
                                #  - Bit 3: \a OP
uint8 link_up                   # true, if at least one Ethernet link is up.
uint32 working_counter          # Value of the last working counter.
uint8 wc_state                  # Working counter interpretation.
                                # 0(EC_WC_ZERO): No registered process data were exchanged.
                                # 1(EC_WC_INCOMPLETE): Some of the registered process data were exchanged.
                                # 2(EC_WC_COMPLETE): All registered process data were exchanged.
uint32 redundancy_active        # Redundant link is in use.