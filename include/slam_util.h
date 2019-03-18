/*******************************************************
 * Copyright (C) 2019, lingy17@mails.tsinghua.edu.cn
 * 
 * 
 * Licensed under the GNU General Public License v3.0;
 * you may not use this file except in compliance with the License.
 *******************************************************/



#pragma once

#define ANSI_COLOR_BLACK            0
#define ANSI_COLOR_RED              1
#define ANSI_COLOR_GREEN            2
#define ANSI_COLOR_YELLOW           3
#define ANSI_COLOR_BLUE             4
#define ANSI_COLOR_MAGENTA          5
#define ANSI_COLOR_CYAN             6
#define ANSI_COLOR_WHITE            7
#define ANSI_COLOR_DEFAULT          9

#define ANSI_FG                     30
#define ANSI_BG                     40

#define ANSI_FORMAT_BOLD_ON         1
#define ANSI_FORMAT_ITALIC_ON       3
#define ANSI_FORMAT_UNDERLINE_ON    4
#define ANSI_FORMAT_INVERSE_ON      7
#define ANSI_FORMAT_STRIKE_ON       9

#define ANSI_FORMAT_BOLD_OFF        22
#define ANSI_FORMAT_ITALIC_OFF      23
#define ANSI_FORMAT_UNDERLINE_OFF   24
#define ANSI_FORMAT_INVERSE_OFF     27
#define ANSI_FORMAT_STRIKE_OFF      29

#define ANSI_SET(file, val) fprintf((file), "%c[%dm", 27, (val))
#define MYFILE (strrchr(__FILE__,'/') ? strrchr(__FILE__,'/')+1:__FILE__)
#define WHR_STR "[ %s:%04d ]: "
#define WHR_ARG MYFILE,__LINE__


#define Fatal(args...) do {                         \
    ANSI_SET(stderr, ANSI_COLOR_RED+ANSI_FG);       \
    fprintf(stderr, WHR_STR, WHR_ARG);              \
    fprintf(stderr, args);                          \
    ANSI_SET(stderr,0);                             \
    exit(1);                                        \
} while (0)

#define Warn(args...) do {                          \
    ANSI_SET(stderr, ANSI_COLOR_YELLOW+ANSI_FG);    \
    fprintf(stderr, WHR_STR, WHR_ARG);              \
    fprintf(stderr, args);                          \
    ANSI_SET(stderr,0);                             \
} while (0)

#define Info(args...) do {                          \
    ANSI_SET(stdout, ANSI_COLOR_GREEN+ANSI_FG);     \
    fprintf(stdout, WHR_STR, WHR_ARG);              \
    fprintf(stdout, args);                          \
    ANSI_SET(stderr,0);                             \
} while (0)
