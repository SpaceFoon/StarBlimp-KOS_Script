run "./aos/ag_legend.ks".

set V0 to GetVoice(0).
// set buttons to terminal:input:buttons. // Not needed

function airship_main {
    set isdone to 0.
    set selag to 1.
    set answer to 0.

    // Get AG keys that are scalars (single AGs)
    set ag_keys to AGX_LEGEND:keys.
    set ag_list to list().
    for key in ag_keys {
        if AGX_LEGEND[key]:typename = "Scalar" {
            ag_list:add(key).
        }
    }
    set max_sel to ag_list:length.
    function airship_gui {
        clearscreen.
        // Paging and layout calculations
        local total to ag_list:length.
        local entries_per_page to 10.
        local rows_per_col to 5.
        local col_width to 26.
        local label_width to col_width - 6.
        local max_page to 0.
        if total > 0 {
            set max_page to floor((total - 1) / entries_per_page).
        }
        local page to floor((selag - 1) / entries_per_page).
        if selag > total {
            set page to max_page.
        }
        if page < 0 { set page to 0. }
        if page > max_page { set page to max_page. }
        local total_pages to max_page + 1.
        if total_pages < 1 { set total_pages to 1. }
        local page_label to "Page " + (page + 1):tostring() + " / " + total_pages:tostring().
        local exit_hint to "(Q to exit)".
        local header_line to page_label + "   " + exit_hint.
        local start_idx to page * entries_per_page.

        print "################################################################################".
        print "|                                                                              |".
        print "|          _________________________________________________________           |".
        print "|          |   AIRSHIP OS 3.1                               - 0 X |            |".
        print "|          |¯¯¯¯¯¯¯¯¯¯¯¯¯¯¯¯¯¯¯¯¯¯¯¯¯¯¯¯¯¯¯¯¯¯¯¯¯¯¯¯¯¯¯¯¯¯¯¯¯¯¯¯¯¯¯|           |".
        print "|          |                   Action Groups Control               |           |".
        print "|          |   ---Select Operation---                              |           |".
        print "|          |   " + header_line:padright(55) + "|           |".
        print "|          |                                                       |           |".

        from {local row is 0.} until row >= rows_per_col step {set row to row + 1.} do {
            local left_str to " ":padright(col_width).
            local right_str to " ":padright(col_width).
            local left_idx to start_idx + row.
            if left_idx < total {
                local num to left_idx + 1.
                local name to ag_list[left_idx].
                if name:length > label_width {
                    set name to name:substring(0,label_width).
                }
                local marker to " ".
                if num = selag { set marker to ">". }
                set left_str to marker + num:tostring():padleft(3) + ". " + name:padright(label_width).
            }
            local right_idx to start_idx + row + rows_per_col.
            if right_idx < total {
                local num to right_idx + 1.
                local name to ag_list[right_idx].
                if name:length > label_width {
                    set name to name:substring(0,label_width).
                }
                local marker to " ".
                if num = selag { set marker to ">". }
                set right_str to marker + num:tostring():padleft(3) + ". " + name:padright(label_width).
            }
            print "|          |" + left_str + "  " + right_str + "|           |".
        }
        print "|          |                                                       |           |".
        print "|          |_______________________________________________________|           |".
        print "|                                                                              |".
        print "|______________________________________________________________________________|".
        print "|  START |                                                           " + time:clock + "  |".
        print "################################################################################".
        print selag at (45,7).
    }
    function airship_answer {
        if answer <= ag_list:length {
            local key to ag_list[answer - 1].
            AG_TOGGLE(AGX_LEGEND[key]).
            print key + " toggled." at (20,16).
            wait 0.5.
            airship_gui().
        }
    }
    set selag to 1.
    airship_gui().
    airship_gui().
    until isdone > 0 {
        set ch to terminal:input:getchar().
        if ch = terminal:input:DOWNCURSORONE {
            set selag to selag + 1.
            if selag > max_sel {
                set selag to 1.
            }
            set ch to "".
            airship_gui().
        }
        if ch = terminal:input:UPCURSORONE {
            set selag to selag - 1.
            if selag < 1 {
                set selag to max_sel.
            }
            set ch to "".
            airship_gui().
        }
        if ch = terminal:input:enter {
            V0:PLAY(NOTE(440,0.5)).
            set answer to selag.
            set ch to "".
            airship_answer().
        }
        if ch = "q" or ch = "Q" {
            print "Exiting..." at (30,16).
            set isdone to 1.
            set ch to "".
            wait 0.5.
        }
        // Number keys for direct selection on current page
        local total to ag_list:length.
        local page_base to 0.
        if total > 0 {
            if selag > total {
                set page_base to floor((total - 1) / 10) * 10.
            } else {
                set page_base to floor((selag - 1) / 10) * 10.
            }
        }
        local key_choice to 0.
        if ch = "1" {
            set key_choice to page_base + 1.
            if key_choice <= max_sel {
                set selag to key_choice.
                airship_gui().
            }
            set ch to "".
        }
        if ch = "2" {
            set key_choice to page_base + 2.
            if key_choice <= max_sel {
                set selag to key_choice.
                airship_gui().
            }
            set ch to "".
        }
        if ch = "3" {
            set key_choice to page_base + 3.
            if key_choice <= max_sel {
                set selag to key_choice.
                airship_gui().
            }
            set ch to "".
        }
        if ch = "4" {
            set key_choice to page_base + 4.
            if key_choice <= max_sel {
                set selag to key_choice.
                airship_gui().
            }
            set ch to "".
        }
        if ch = "5" {
            set key_choice to page_base + 5.
            if key_choice <= max_sel {
                set selag to key_choice.
                airship_gui().
            }
            set ch to "".
        }
        if ch = "6" {
            set key_choice to page_base + 6.
            if key_choice <= max_sel {
                set selag to key_choice.
                airship_gui().
            }
            set ch to "".
        }
        if ch = "7" {
            set key_choice to page_base + 7.
            if key_choice <= max_sel {
                set selag to key_choice.
                airship_gui().
            }
            set ch to "".
        }
        if ch = "8" {
            set key_choice to page_base + 8.
            if key_choice <= max_sel {
                set selag to key_choice.
                airship_gui().
            }
            set ch to "".
        }
        if ch = "9" {
            set key_choice to page_base + 9.
            if key_choice <= max_sel {
                set selag to key_choice.
                airship_gui().
            }
            set ch to "".
        }
        if ch = "0" {
            set key_choice to page_base + 10.
            if key_choice <= max_sel {
                set selag to key_choice.
                airship_gui().
            }
            set ch to "".
        }
    }
}

airship_main().