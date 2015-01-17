
#include <stdio.h>
#include <getopt.h>
#include <errno.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <stdlib.h>
#include <termios.h>
#include <string.h>
#include <signal.h>
#include <sys/system_properties.h>
#define PROPERTY_VALUE_MAX  PROP_VALUE_MAX

//int init_baudrate = 115200;               // Motorola DroidX2 uses a startup baudrate of 3.5 MHz
//int new_baudrate  = 0;//3000000;//0;

#ifndef N_HCI
#define N_HCI  15
#endif

#define HCIUARTSETPROTO    _IOW('U', 200, int)
#define HCIUARTGETPROTO    _IOR('U', 201, int)
#define HCIUARTGETDEVICE  _IOR('U', 202, int)

#define HCI_UART_H4    0
#define HCI_UART_BCSP  1
#define HCI_UART_3WIRE  2
#define HCI_UART_H4DS  3
#define HCI_UART_LL    4

// property_set ("ctl.start", "hciattach")
// property_set ("ctl.stop", "hciattach")

int exiting =       0;
int uart_fd =       -1;
int hcdfile_fd =    -1;
int bdaddr_flag =   0;
int enable_lpm =    0;
int enable_hci =    0;

//int disable_baudrate_next = 1;//0;

struct termios termios = {0};
unsigned char hci_recv_buf [1024] = {0};             // Should only need 259 bytes ?

// Sending order
unsigned char hci_reset [] =             { 0, 0, 0, 0, 0x01, 0x03, 0x0c, 0x00 };                                     // OGF:    3    OCF:    3   (Host Controller & Baseband Commands, Reset)
unsigned char hci_patchram_start [] =    { 0, 0, 0, 0, 0x01, 0x2e, 0xfc, 0x00 };                                     // OGF: 0x3f    OCF: 0x2e
    // After this many packets sent from BCM4325D1_004.002.004.0218.0248.hcd: 48, 259,..., 114, 8
unsigned char hci_baudrate_reset [] =    { 0, 0, 0, 0, 0x01, 0x18, 0xfc, 0x06, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 }; // OGF: 0x3f    OCF: 0x18   Parameter Total Length:  6
unsigned char hci_bdaddr_set [] =        { 0, 0, 0, 0, 0x01, 0x01, 0xfc, 0x06, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 }; // OGF: 0x3f    OCF: 0x01   Parameter Total Length:  6

// Sent last normally, but optional:
unsigned char hci_lpm_set [] =           { 0, 0, 0, 0, 0x01, 0x27, 0xfc, 0x0c, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01,   // OGF: 0x3f    OCF: 0x27   Parameter Total Length: 12
                                            0x01, 0x00, 0x00, 0x00, 0x00, 0x00 };

// Each command is assigned a 2 byte Opcode used to uniquely identify different types of commands.
// The Opcode parameter is divided into two fields, called the OpCode Group Field (OGF) and OpCode Command Field (OCF).
// The OGF occupies the upper 6 bits of the Opcode, while the OCF occupies the remaining 10 bits. The OGF of 0x3F is reserved for vendor-specific debug commands. 

int uart_recv (int fd, unsigned char *buf, int flush);
int uart_send (unsigned char *buf, int len);
int hci_xact  (char *buf, int len);


/* Default power up permissions:

drwxr-xr-x   14 root     root          3240 May 16 05:04 dev
drwxrwxr-x    2 root     net_raw        220 May 16 05:04 socket

*/

#define ERROR_CODE_NUM 56
static char *error_code_str[ERROR_CODE_NUM + 1] = {
    "Success",
    "Unknown HCI Command",
    "Unknown Connection Identifier",
    "Hardware Failure",
    "Page Timeout",
    "Authentication Failure",
    "PIN or Key Missing",
    "Memory Capacity Exceeded",
    "Connection Timeout",
    "Connection Limit Exceeded",
    "Synchronous Connection to a Device Exceeded",
    "ACL Connection Already Exists",
    "Command Disallowed",
    "Connection Rejected due to Limited Resources",
    "Connection Rejected due to Security Reasons",
    "Connection Rejected due to Unacceptable BD_ADDR",
    "Connection Accept Timeout Exceeded",
    "Unsupported Feature or Parameter Value",
    "Invalid HCI Command Parameters",
    "Remote User Terminated Connection",
    "Remote Device Terminated Connection due to Low Resources",
    "Remote Device Terminated Connection due to Power Off",
    "Connection Terminated by Local Host",
    "Repeated Attempts",
    "Pairing Not Allowed",
    "Unknown LMP PDU",
    "Unsupported Remote Feature / Unsupported LMP Feature",
    "SCO Offset Rejected",
    "SCO Interval Rejected",
    "SCO Air Mode Rejected",
    "Invalid LMP Parameters",
    "Unspecified Error",
    "Unsupported LMP Parameter Value",
    "Role Change Not Allowed",
    "LMP Response Timeout",
    "LMP Error Transaction Collision",
    "LMP PDU Not Allowed",
    "Encryption Mode Not Acceptable",
    "Link Key Can Not be Changed",
    "Requested QoS Not Supported",
    "Instant Passed",
    "Pairing with Unit Key Not Supported",
    "Different Transaction Collision",
    "Reserved",
    "QoS Unacceptable Parameter",
    "QoS Rejected",
    "Channel Classification Not Supported",
    "Insufficient Security",
    "Parameter out of Mandatory Range",
    "Reserved",
    "Role Switch Pending",
    "Reserved",
    "Reserved Slot Violation",
    "Role Switch Failed",
    "Extended Inquiry Response Too Large",
    "Simple Pairing Not Supported by Host",
    "Host Busy - Pairing",
};

//static char *status2str (uint8_t status) {
char *hci_err_get (uint8_t status) {
  char *str;
  if (status <= ERROR_CODE_NUM)
    str = error_code_str[status];
  else
    str = "Unknown HCI Error";
  return (str);
}



#define MAX_HCI  264   // 8 prepended bytes + 255 max bytes HCI data/parameters + 1 trailing byte to align


// Unix datagrams requires other write permission for /dev/socket, or somewhere else (ext not FAT on sdcard) writable.

//#define CS_AF_UNIX        // Use network sockets to avoid filesystem permission issues.
#define CS_DGRAM

#ifdef  CS_AF_UNIX
#define DEF_API_SRVSOCK    "/dev/socket/srv_sprt"
#define DEF_API_CLISOCK    "/dev/socket/cli_sprt"
char api_srvsock[DEF_BUF] =DEF_API_SRVSOCK;
char api_clisock[DEF_BUF] =DEF_API_CLISOCK;
#endif

#ifdef  CS_AF_UNIX
#include <sys/un.h>
#define CS_FAM   AF_UNIX
#else
#include <netinet/in.h>
#include <netdb.h> 
#define CS_PORT    2112
#define CS_FAM   AF_INET
#endif

#ifdef  CS_DGRAM
#define CS_SOCK_TYPE    SOCK_DGRAM
#else
#define CS_SOCK_TYPE    SOCK_STREAM
#endif

//unsigned char res_buf [MAX_HCI] = {0};

int do_daemon_hci ( char *cmd_buf, int cmd_len, char *res_buf, int res_max ) {
  if (cmd_len==1 && cmd_buf [0] == 0x73) {
    logd ("do_daemon_hci got ready inquiry");
    res_buf [0] = cmd_buf [0];
    return (1);
  }
  else if (cmd_len==1 && cmd_buf [0] == 0x77) {
    logd ("do_daemon_hci got debug_extra off");
    res_buf [0] = cmd_buf [0];
    hci_dbg = reg_dbg = evt_dbg = 0;
    return (1);
  }
  else if (cmd_len==1 && cmd_buf [0] == 0x78) {
    logd ("do_daemon_hci got debug_extra on");
    res_buf [0] = cmd_buf [0];
    hci_dbg = reg_dbg = evt_dbg = 1;
    return (1);
  }
  else if (cmd_len==1 && cmd_buf [0] == 0x7f) {
    logd ("do_daemon_hci got stop");
    res_buf [0] = cmd_buf [0];
    exiting = 1;
    return (1);
  }

  int hx_ret = hci_xact (cmd_buf, cmd_len);                     // Do HCI transaction
  if (hx_ret < 8 || hx_ret > 270) {
    hci_recv_buf [0] = 0xff; // Error
    return (8);
  }
  hci_recv_buf [0] = 0;
  memcpy (res_buf, hci_recv_buf, hx_ret);
  //hex_dump ("aaaa", 32, hci_recv_buf, hx_ret);
  //hex_dump ("bbbb", 32, res_buf, hx_ret);
  
  if (res_buf [7])
    loge ("do_daemon_hci hci err: %d %s", res_buf [7], hci_err_get (res_buf [7]));
  
  return (hx_ret);
}

//char stop_resp[] ={2, 0xff, 0, 0, 0, 0, 0, 0};
//char err_resp[] = {2, 0xfe, 0, 0, 0, 0, 0, 0};

int do_server () {
  int sockfd = -1, newsockfd = -1, cmd_len = 0, ctr = 0;
  socklen_t cli_len = 0, srv_len = 0;
#ifdef  CS_AF_UNIX
  struct sockaddr_un  cli_addr = {0}, srv_addr = {0};
  srv_len = strlen (srv_addr.sun_path) + sizeof (srv_addr.sun_family);
#else
  struct sockaddr_in  cli_addr = {0}, srv_addr = {0};
  //struct hostent *hp;
#endif
  char cmd_buf [DEF_BUF] ={0};

  //system("chmod 666 /dev");            // !! Need su if in JNI
  //system("chmod 666 /dev/socket");

#ifdef  CS_AF_UNIX
  unlink (api_srvsock);
#endif
  if ((sockfd = socket (CS_FAM,CS_SOCK_TYPE, 0)) < 0) {
    loge ("do_server socket  errno: %d", errno);
    return (-1);
  }

  bzero((char *) &srv_addr, sizeof (srv_addr));
#ifdef  CS_AF_UNIX
  srv_addr.sun_family = AF_UNIX;
  strncpy (srv_addr.sun_path, api_srvsock, sizeof (srv_addr.sun_path));
  srv_len = strlen (srv_addr.sun_path) + sizeof (srv_addr.sun_family);
#else
  srv_addr.sin_family=AF_INET;
  srv_addr.sin_addr.s_addr = htonl (INADDR_LOOPBACK); //INADDR_ANY;
  //hp = gethostbyname("localhost");
  //if (hp== 0) {
  //  loge ("Error gethostbyname  errno: %d", errno);
  //  return (-2);
  //}
  //bcopy((char *)hp->h_addr, (char *)&srv_addr.sin_addr, hp->h_length);
  srv_addr.sin_port = htons (CS_PORT);
  srv_len = sizeof (struct sockaddr_in);
#endif

#ifdef  CS_AF_UNIX
logd ("srv_len: %d  fam: %d  path: %s", srv_len, srv_addr.sun_family, srv_addr.sun_path);
#else
logd ("srv_len: %d  fam: %d  addr: 0x%x  port: %d", srv_len, srv_addr.sin_family, ntohl (srv_addr.sin_addr.s_addr), ntohs (srv_addr.sin_port));
#endif
  if (bind (sockfd,(struct sockaddr *)&srv_addr, srv_len) < 0) {
    loge ("Error bind  errno: %d", errno);
#ifdef  CS_AF_UNIX
    return (-3);
#endif
#ifdef CS_DGRAM
    return (-3);
#endif
    loge ("Inet stream continuing despite bind error");      // OK to continue w/ Internet Stream
  }

// Get command from client
#ifndef CS_DGRAM
  if (listen(sockfd, 5)) {                           // Backlog= 5; likely don't need this
    loge ("Error listen  errno: %d", errno);
    return (-4);
  }
#endif

  logd ("do_server Ready");

  while (!exiting) {
    bzero((char *) &cli_addr, sizeof (cli_addr));                        // ?? Don't need this ?
    //cli_addr.sun_family = CS_FAM;                                     // ""
    cli_len = sizeof (cli_addr);

    //logd ("ms_get: %d",ms_get ());
#ifdef  CS_DGRAM
    cmd_len = recvfrom(sockfd, cmd_buf, sizeof (cmd_buf), 0,(struct sockaddr *)&cli_addr,&cli_len);
    if (cmd_len <= 0) {
      loge ("Error recvfrom  errno: %d", errno);
      ms_sleep (100);   // Sleep 0.1 second
      continue;
    }
  #ifndef CS_AF_UNIX
// !! 
    if ( cli_addr.sin_addr.s_addr != htonl (INADDR_LOOPBACK) ) {
      loge ("Unexpected suspicious packet from host %s",inet_ntop(cli_addr.sin_addr.s_addr)); //inet_ntoa(cli_addr.sin_addr.s_addr));
    }
  #endif
#else
    newsockfd = accept(sockfd,(struct sockaddr *)&cli_addr,&cli_len);
    if (newsockfd < 0) {
      loge ("Error accept  errno: %d", errno);
      ms_sleep (100);   // Sleep 0.1 second
      continue;
    }
  #ifndef  CS_AF_UNIX
// !! 
    if ( cli_addr.sin_addr.s_addr != htonl (INADDR_LOOPBACK) ) {
      loge ("Unexpected suspicious packet from host %s",inet_ntop(cli_addr.sin_addr.s_addr)); //inet_ntoa(cli_addr.sin_addr.s_addr));
    }
  #endif
    cmd_len = read (newsockfd, cmd_buf, sizeof (cmd_buf));
    if (cmd_len <= 0) {
      loge ("Error read  errno: %d", errno);
      ms_sleep (100);   // Sleep 0.1 second
      close (newsockfd);
      ms_sleep (100);   // Sleep 0.1 second
      continue;
    }
#endif

#ifdef  CS_AF_UNIX
//logd ("cli_len: %d  fam: %d  path: %s",cli_len,cli_addr.sun_family,cli_addr.sun_path);
#else
//logd ("cli_len: %d  fam: %d  addr: 0x%x  port: %d",cli_len,cli_addr.sin_family, ntohl (cli_addr.sin_addr.s_addr), ntohs (cli_addr.sin_port));
#endif
    //hex_dump ("", 32, cmd_buf, n);

    unsigned char res_buf [MAX_HCI] = {0};
    int res_len= 0;
/* OLD WAY !!!
    if (cmd_len==2 && cmd_buf [0] ==2 && cmd_buf [1] == 0xff) {
      res_len=2;                        //response= stop_resp;
      exiting=1;
    }
    else if (cmd_len==2 && cmd_buf [0] ==3 && cmd_buf [1] == 0xff) {
      res_len=2;
      hci_dbg = reg_dbg = evt_dbg = 1;
    }
    else if (cmd_len==2 && cmd_buf [0] ==4 && cmd_buf [1] == 0xff) {
      res_len=2;
      hci_dbg = reg_dbg = evt_dbg = 0;
    }
// Process command
    else {
*/
      res_len = do_daemon_hci ( cmd_buf, cmd_len, res_buf, sizeof (res_buf));    // Do HCI (or other) function
      //logd ("do_server do_daemon_hci res_len: %d", res_len);
      if (res_len < 0) {  // If error
        res_len = 2;
        res_buf [0] = 0xff;
        res_buf [1] = 0xff;
      }
      //hex_dump ("", 32, res_buf, res_len);
//    }

// Send response
#ifdef  CS_DGRAM
    if (sendto(sockfd, res_buf, res_len, 0,(struct sockaddr *)&cli_addr,cli_len) != res_len) {
      loge ("Error sendto  errno: %d", errno);
      ms_sleep (100);   // Sleep 0.1 second
    }
#else
    if (write (newsockfd, res_buf, res_len) != res_len) {
      loge ("Error write  errno: %d", errno);
      ms_sleep (100);   // Sleep 0.1 second
    }
    close (newsockfd);
#endif
  }
  close (sockfd);
#ifdef  CS_AF_UNIX
  unlink (api_srvsock);
#endif

  //acc_hci_stop ();

  return (0);//stop_resp);//"server finished end");
}



typedef struct {
  int baudrate;
  int termios_baudrate;
} tbaudrates;

//#define REAL_LOW_BAUDRATES                                            // Doesn't work below 9600
#define LOW_BAUDRATES
tbaudrates baudrates[] = {
#ifdef  REAL_LOW_BAUDRATES
  {      50,      B50 },
  {      75,      B75 },
  {     110,     B110 },
  {     134,     B134 },
  {     150,     B150 },
  {     200,     B200 },
  {     300,     B300 },
  {     600,     B600 },
  {    1200,    B1200 },
  {    1800,    B1800 },
  {    2400,    B2400 },
  {    4800,    B4800 },
#endif
  {    9600,    B9600 },
#ifdef  LOW_BAUDRATES
  {   19200,   B19200 },
  {   38400,   B38400 },
  {   57600,   B57600 },
#endif
  {  115200,  B115200 },
  {  230400,  B230400 },
  {  460800,  B460800 },
  {  500000,  B500000 },
  {  576000,  B576000 },
  {  921600,  B921600 },
  { 1000000, B1000000 },
  { 1152000, B1152000 },
  { 1500000, B1500000 },
  { 2000000, B2000000 },
  { 2500000, B2500000 },
  { 3000000, B3000000 },
  { 3500000, B3500000 },
  { 4000000, B4000000 },
};

//#define ALL_RATES
int rate_idx = 0;
/*
int uart_baudrate_next () {
  if (disable_baudrate_next)
    return (0);
  logd ("uart_baudrate_next to next");
#ifdef  ALL_RATES
  if (rate_idx >= (sizeof (baudrates) / sizeof (tbaudrates)) ) {
    rate_idx = 0;
  }
  uart_baudrate_set (baudrates [rate_idx].baudrate);
  rate_idx ++;
#else
  int baudrate = uart_baudrate_get ();
  if (baudrate == 9600)
    uart_baudrate_set (115200);
  else if (baudrate == 115200)
    uart_baudrate_set (3000000);
  else
    uart_baudrate_set (9600);
#endif
  return (0);
}
*/


int termios_baudrate_get (int baudrate) {                               // Convert normal numeric baudrate to termios baudrate.
  int idx = 0;
  for (idx = 0; idx < (sizeof (baudrates) / sizeof (tbaudrates)); idx ++)
    if (baudrates [idx].baudrate == baudrate)
      return (baudrates [idx].termios_baudrate);
  return (0);
}

int baudrate_get (int termios_baudrate) {                       // Convert termios baudrate to normal numeric baudrate.
  int idx= 0;
  for (idx = 0; idx < (sizeof (baudrates) / sizeof (tbaudrates)); idx ++)
    if (baudrates [idx].termios_baudrate == termios_baudrate)
      return (baudrates [idx].baudrate);
  return (0);
}

int uart_baudrate_set (int baudrate) {                                  // Set ONLY port baudrate (doesn't reset chip baudrate)
  logd ("baudrate_set: %d", baudrate);
  int termios_baudrate = termios_baudrate_get (baudrate);
  if (termios_baudrate) {
    cfsetospeed (& termios, termios_baudrate);
    cfsetispeed (& termios, termios_baudrate);
    tcsetattr (uart_fd, TCSANOW, & termios);
    return (0);
  }
  loge ("baudrate_set invalid: %d", baudrate);
  return (-1);
}

int uart_baudrate_get () {                                              // Get current port baudrate
  speed_t termios_osp = cfgetospeed (& termios);
  speed_t termios_isp = cfgetispeed (& termios);
  logd ("uart_baudrate_get termios_osp: %d  termios_isp: %d", termios_osp, termios_isp);
  int baudrate = baudrate_get (termios_osp);
  logd ("uart_baudrate_get: %d", baudrate);
  return (baudrate);
}
/* No longer used ?
int baudrate_reset (int baudrate) {                                     // Reset the HCI interface baudrate, then set the port baudrate; Assumes we have valid communications first
  logd ("baudrate_reset: %d", baudrate);
  int ret = -1;
  int termios_baudrate = termios_baudrate_get (baudrate);
  if (termios_baudrate) {                                               // If valid baudrate
    hci_baudrate_reset [13] = (unsigned char) (baudrate >> 24);
    hci_baudrate_reset [12] = (unsigned char) (baudrate >> 16);
    hci_baudrate_reset [11] = (unsigned char) (baudrate >> 8);
    hci_baudrate_reset [10] = (unsigned char) (baudrate & 0xFF);
    if (hci_xact (hci_baudrate_reset, sizeof (hci_baudrate_reset)) < 0)
      return (-1);
    ret = uart_baudrate_set (baudrate);                                 // Validated so should never return an error
    return (ret);
  }
  loge ("baudrate_reset invalid: %d",baudrate);
  return (-1);
}
*/
int start_baudrate = -1;

int uart_init (int set_baudrate) {
  logd ("uart_init baud: ", set_baudrate);

  tcflush (uart_fd, TCIOFLUSH);
  tcgetattr (uart_fd, & termios);

  cfmakeraw (& termios);
/* Equivalent without cfmakeraw()
  termios.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL | IXON);
  termios.c_oflag &= ~OPOST;
  termios.c_lflag &= ~(ECHO | ECHONL | ICANON | ISIG | IEXTEN);
  termios.c_cflag &= ~(CSIZE | PARENB);
  termios.c_cflag |= CS8;
*/

  //termios.c_cflag |= CSTOPB;

  termios.c_cflag |= CRTSCTS;                       // RTS - CTS flow control
  tcsetattr (uart_fd, TCSANOW, & termios);
  tcflush (uart_fd, TCIOFLUSH);
  tcsetattr (uart_fd, TCSANOW, & termios);
  tcflush (uart_fd, TCIOFLUSH);
  tcflush (uart_fd, TCIOFLUSH);

  //int baudrate = uart_baudrate_get ();  // Was/IS just to display...
  start_baudrate = uart_baudrate_get ();

  // When started by Android BT start, baudrate = 115200, otherwise 9600  
  //uart_baudrate_set (9600);
  //uart_baudrate_set (115200);
  //uart_baudrate_set (3000000);

  //if (init_baudrate)
  //  uart_baudrate_set (init_baudrate);
  //else
  //  uart_baudrate_set (baudrate);                                       // ?? Set to current baudrate ??

//uart_baudrate_set (start_baudrate);
  uart_baudrate_set (set_baudrate);
  //uart_baudrate_set (baudrate);

  tcflush (uart_fd, TCIOFLUSH);                                          // !! Flush RX + Tx to prevent reset_start () bogus first response rx byte 0xf0
  tcflush (uart_fd, TCIFLUSH);                                           // !! Flush RX      to prevent reset_start () bogus first response rx byte 0xf0

  return (0);
}

//int xact_ms_sleep = 0;//2000;

int reset_start () {

  tcflush (uart_fd, TCIOFLUSH);                                         // !! Flush RX + Tx to prevent reset_start () bogus first response rx byte 0xf0
  tcflush (uart_fd, TCIFLUSH);                                          // !! Flush RX      to prevent reset_start () bogus first response rx byte 0xf0

  logd ("reset_start");

  //xact_ms_sleep = 2000;
  if (hci_xact (hci_reset, sizeof (hci_reset)) < 0) {                   // Send the reset. If error...
    //xact_ms_sleep = 0;
    if (hci_xact (hci_reset, sizeof (hci_reset)) < 0) {                 // Send a second reset. Second attempt works better when switching baudrates.
      return (-1);                                                      // If error, return error
    }
  }
  //xact_ms_sleep = 0;
  return (0);
}

int hcd_main_set_done = 0;
char hcd_main [DEF_BUF] = "/mnt/sdcard/sprt/broadcomp.hcd";

char * hcd_list [] = {
  hcd_main,
};  
char hcd_buf [DEF_BUF] = {0};

char * hcd_get () {
  int ctr = 0, ret = 0;
  char *hcd = NULL;

  hcd_buf [0] = 0;

  if (hcd_main_set_done == 0) {
    hcd_main_set_done = 1;
    char * ext_storage = getenv ("EXTERNAL_STORAGE");
    if (ext_storage) {
      strlcpy (hcd_main, ext_storage, sizeof (hcd_main));
      strlcat (hcd_main, "/sprt/broadcomp.hcd", sizeof (hcd_main));
    }
    else {
      strlcpy (hcd_main, "/mnt/sdcard/sprt/broadcomp.hcd", sizeof (hcd_main));
    }
  }

  for (ctr = 0; ctr < (sizeof (hcd_list) / sizeof (char *)); ctr ++) {  // For all entries in hcd file list
    hcd = hcd_list [ctr];
    if (access_file_get (hcd, O_RDONLY)) {                              // If we have this hcd file and accessible...
      logd ("hcd_get from hcd list     have HCD: %s", hcd);
      return (hcd);
    }
    else {
      logd ("hcd_get no                     HCD: %s", hcd);
    }
  }
  hcd_buf [0] = 0;
  ret = hcd_file_find (hcd_buf, sizeof (hcd_buf));
  if (ret && access_file_get (hcd_buf, O_RDONLY)) {                            // If we have this hcd file and accessible...
    logd ("hcd_get from find         have HCD: %s", hcd_buf);
    return (hcd_buf);
  }
  loge ("hcd_get no HCD found");
  return (NULL);
}

int patchram_set () {
  unsigned char patchram_send_buf [1024] = {0};                         // Should only need 259 bytes or so
  int len= 0, ret = 0;
  logd ("patchram_set");

  char * hcd = hcd_get ();                                              // Get *.hcd file
  if (! hcd)
    return (-1);
  if ((hcdfile_fd = open (hcd, O_RDONLY)) < 0) {                        // Open hcd file
    loge ("patchram_set open errno: %d", errno);
    return (-5);
  }
  logd ("patchram_set fd: %d", hcdfile_fd);

  ret = hci_xact (hci_patchram_start, sizeof (hci_patchram_start));      // Start patchram
  if (ret < 0) {
    loge ("patchram_set hci_xact 1 error: %d", ret);
    return (-1);
  }
/* HTC One CM 10.2 Aug 7, 2013:
a cat vendor/firmware/bcm4335_prepatch.hcd|hd
0000:   4c fc 3c    00 02 0d 00 70  b5 0c 49 4c f6 20 30 8a
                    f7 bb f9 01 28 0d d1 8a  f7 80 f9 08 b1 04 25 00
                    e0 05 25 00 24 03 e0 8a  f7 74 f9 64 1c e4 b2 ac
                    42 f9 d3 bd e8 70 40 8a  f7 7f b9 b0 9b 04 00

003F:   4e fc 04    00 02 0d 00


a cat system/etc/firmware/BCM4335B0_002.001.006.0092.0093.hcd|hd|head -32
00000000  4c fc 8b      00 65 21 00 01  06 00 ea 44 72 42 04 1a
00000010  c1 5e 00 ef 63 15 83 a4  ef 63 15 83 a4 37 02 56
00000020  02 56 02 57 02 57 02 58  02 58 02 59 02 59 02 5a
00000030  02 5a 02 5b 02 5b 02 5c  02 5c 02 5d 02 5d 02 5e
00000040  02 5e 02 5f 02 5f 02 60  02 60 02 61 02 61 02 62
00000050  02 62 02 63 02 63 02 64  02 64 02 65 02 65 02 66
00000060  02 66 02 67 02 67 02 68  02 68 02 69 02 69 28 1f
00000070  0d 0a fd 04 00 ff ff ff  ff 40 06 00 00 00 07 b0
00000080  35 43 02 0d 0a 00 87 65  21 00 00 00 00 00

008E      00 00

00000090  4c fc ff 87 65 21 00 41  36 00 42 43 4d 34 33 33  |L...e!.A6.BCM433|
000000a0  35 42 30 20 33 37 2e 34  4d 48 7a 20 57 4c 42 47  |5B0 37.4MHz WLBG|
000000b0  41 46 45 4d 5f 61 6e 61  64 69 67 69 63 73 20 49  |AFEM_anadigics I|
000000c0  32 53 2d 53 6c 61 76 65  20 48 54 43 2d 4d 37 00  |2S-Slave HTC-M7.|
000000d0  c0 f8 01 54 f9 31 00 ff  ff 00 00 08 08 00 00 e4
000000e0  fc 31 00 ff ff 00 00 28  c8 00 00 30 04 60 00 ff
000000f0  00 00 00 01 00 00 00 60  04 60 00 ff 00 00 00 0c
00000100  00 00 00 74 04 60 00 ff  00 00 00 03 00 00 00 dc
00000110  04 60 00 ff 00 00 00 1a  00 00 00 70 05 60 00 ff
00000120  00 00 00 0d 00 00 00 84  05 60 00 ff 00 00 00 33
00000130  00 00 00 20 06 60 00 ff  00 00 00 17 00 00 00 24
00000140  06 60 00 ff 00 00 00 00  00 00 00 34 06 60 00 ff
00000150  00 00 00 09 00 00 00 40  06 60 00 ff 00 00 00 78
00000160  00 00 00 38 07 60 00 ff  00 00 00 91 00 00 00 64
00000170  07 60 00 ff 00 00 00 58  00 00 00 b0 02 60 00 ff
00000180  00 00 00 48 00 00 00 dc  02 60 00 ff 00 00 00 48
00000190  00 00

0192:   4c fc ff    82 66 21  00 00 64 00 64 00 ff 00
000001a0  00 00 01 00 00 00 90 00  64 00 ff 00 00 00 29 00
000001b0  00 00 cc 00 64 00 ff 00  00 00 2b 00 00 00 d0 00
000001c0  64 00 ff 00 00 00 43 00  00 00 d4 00 64 00 ff 00
000001d0  00 00 30 00 00 00 d8 00  64 00 ff 00 00 00 79 00
000001e0  00 00 e0 00 64 00 ff 00  00 00 1e 00 00 00 fc 00
000001f0  64 00 ff 00 00 00 80 00  00 00 24 01 64 00 ff 00

*/
/*  /system/bin/BCM4325D1_004.002.004.0218.0248.hcd         12551 bytes (0x3107)

   47:  0x3f 0x4c:   0x2c (44)
12384:  0x3f 0x4c:   48 * 255   -> 258
  113:  0x3f 0x4c:   0x6e (110)
    7:  0x3f 0x4e    ff ff ff ff
----------
12551

    0x0008 6800 - 0x0008 9738 (+ 0x6e) [0x0008 97a5]
    length: 0x2fa6 = 12197 bytes

12197 = 12551 - 354
50 * 3 = 150

*/
  //ret = read (uart_fd, &patchram_send_buf [4], 2);                     // Read 2 bytes from UART (only uses 1st, read () below writes over other)
  //ret = tmo_read (uart_fd, &patchram_send_buf [4], 2, 5000, 0);        // W/ timeout 5 s, Read 2 bytes from UART (uses neither byte), doing multiple reads if needed

//Always times out now ??
  ret = tmo_read (uart_fd, & patchram_send_buf [4], 2, 2000, 0);        // W/ timeout 2 s, Read 2 bytes from UART (uses neither byte), doing multiple reads if needed
                                                                        // Change timeout to 2 seconds due to Samsung removal for "//for BCM4330B1"
  logd ("patchram_set read 1 ret: %d", ret);

  ms_sleep (50);
  while (ret = read (hcdfile_fd, & patchram_send_buf [5], 3) > 0) {     // Read 3 bytes from hcdfile until returns 0   (!! could mess up!!)
    if (ret < 0) {                                                      // If error...
      loge ("patchram_set read 2 ret: %d  errno: %d", ret, errno);
      return (-1);                                                      // Done w/ error
    }
    if (ret != 1)
      logd ("patchram_set read 2 ret: %d", ret);                        // Always shows 1 ??
    patchram_send_buf [4] = 0x01;
    len = patchram_send_buf [7];
    ret = read (hcdfile_fd, & patchram_send_buf [8], len);              // Read specified length of file
    if (ret < 0) {
      loge ("patchram_set read 3 ret: %d  len: %d  errno: %d", ret, len, errno);
      return (-1);
    }
    logd ("patchram_set read 3 ret: %d  len: %d", ret, len);

    if (hci_xact (patchram_send_buf, len + 8) < 0) {                      // Send to UART
      loge ("patchram_set hci_xact 2 error: %d", ret);
      return (-1);
    }
  }
  logd ("patchram_set read 4 last ret: %d", ret);
  close (hcdfile_fd);
  hcdfile_fd = -1;
  return (0);
}

int bdaddr_set () {
  logd ("bdaddr_set");
  if (hci_xact (hci_bdaddr_set, sizeof (hci_bdaddr_set)) < 0)
    return (-1);
  return (0);
}

int enable_lpm_set () {     // Low power mode aka sleep mode
  logd ("enable_lpm_set");
  if (hci_xact (hci_lpm_set, sizeof (hci_lpm_set)) < 0)
    return (-1);
  return (0);
}

int enable_hci_set () {
  logd ("enable_hci_set");
  int i = N_HCI;
  int proto = HCI_UART_H4;
  if (ioctl (uart_fd, TIOCSETD, &i) < 0) {
    loge ("enable_hci_set ioctl set line discipline errno: %d", errno);
    return (-1);
  }
  if (ioctl (uart_fd, HCIUARTSETPROTO, proto) < 0) {
    loge ("enable_hci_set ioctl set hci protocol errno: %d", errno);
    return (-1);
  }
  return (0);
}

int default_bdaddr_set () {
  int sz= 0, fd = -1;
  char path[PROPERTY_VALUE_MAX] = {0}, bdaddr[18] = {0};
  logd ("default_bdaddr_set");

  __system_property_get ("ro.bt.bdaddr_path", path);
  if (path[0] == 0)
    return (0);     // Common so no error

  fd = open (path, O_RDONLY);
  if (fd < 0) {
    loge ("default_bdaddr_set open: %s  errno: %s (%d)", path, strerror (errno), errno);
    return (-1);
  }

  sz = read (fd, bdaddr, sizeof (bdaddr));
  if (sz < 0) {
    loge ("default_bdaddr_set read: %s  errno: %s (%d)", path, strerror (errno), errno);
    close (fd);
    return (-1);
  } else if (sz != sizeof (bdaddr)) {
    loge ("default_bdaddr_set read: %s  unexpected size: %d", path, sz);
    close (fd);
    return (-1);
  }

  close (fd);
  logd ("default_bdaddr_set bdaddr: %s", bdaddr);

  int tbdaddr[6] = {0};
  int i= 0;
  sscanf(bdaddr, "%02X:%02X:%02X:%02X:%02X:%02X", &tbdaddr[5], &tbdaddr[4], &tbdaddr[3], &tbdaddr[2], &tbdaddr[1], &tbdaddr[0]);
  for (i = 0; i < 6; i++)
    hci_bdaddr_set[8 + i] = tbdaddr[i];
  bdaddr_flag = 1;  

  return (0);
}




unsigned char hci_ver_get[] = { 0, 0, 0, 0, 0x01, 0x01, 0x10, 0x00 };   // HCI Version get

#ifdef  FM_MODE
unsigned char hci_fm_on[] = { 0, 0, 0, 0, 0x01, 0x15, 0xfc, 0x03, 0x00, 0x00, 0x01 };   // power reg 0 = 1
unsigned char hci_ct_on[] = { 0, 0, 0, 0, 0x01, 0x15, 0xfc, 0x03, 0x01, 0x00, 0x04 };   // aud ctrl reg 1 = 4
unsigned char hci_au_on[] = { 0, 0, 0, 0, 0x01, 0x15, 0xfc, 0x03, 0x05, 0x00, 0x5c };   // aud ctrl reg 5 = 5c
unsigned char hci_fa_on[] = { 0, 0, 0, 0, 0x01, 0x15, 0xfc, 0x03, 0x0a, 0x00, 0xb4 };   // freqa 88.5
unsigned char hci_fb_on[] = { 0, 0, 0, 0, 0x01, 0x15, 0xfc, 0x03, 0x0b, 0x00, 0x5f };   // freqb 88.5
unsigned char hci_fp_on[] = { 0, 0, 0, 0, 0x01, 0x15, 0xfc, 0x03, 0x09, 0x00, 0x01 };   // freq preset

unsigned char hci_ra_on[] = { 0, 0, 0, 0, 0x01, 0x0a, 0xfc, 0x09, 0x05, 0xc0, 0x41, 0x0f, 0x00, 0x20, 0x00, 0x00, 0x00 };   // audio routing a
unsigned char hci_rb_on[] = { 0, 0, 0, 0, 0x01, 0x0a, 0xfc, 0x09, 0x05, 0xe4, 0x41, 0x0f, 0x00, 0x00, 0x00, 0x00, 0x00 };   // audio routing b
unsigned char hci_vo_on[] = { 0, 0, 0, 0, 0x01, 0x0a, 0xfc, 0x09, 0x05, 0xe0, 0x41, 0x0f, 0x00, 0x7f, 0x00, 0x00, 0x00 };   // volume 0x7f = half
unsigned char hci_rg_on[] = { 0, 0, 0, 0, 0x01, 0x15, 0xfc, 0x03, 0x0f, 0x01, 0x01 };   // RSSI get


int hci_lens[] = {
    sizeof (hci_fm_on),
    sizeof (hci_ct_on),
    sizeof (hci_au_on),
    sizeof (hci_fa_on),
    sizeof (hci_fb_on),
    sizeof (hci_fp_on),
    sizeof (hci_ra_on),
    sizeof (hci_rb_on),
    sizeof (hci_vo_on),
    sizeof (hci_rg_on),
    sizeof (hci_ver_get),
};
unsigned char * hci_cmds[] = {
    hci_fm_on,
    hci_ct_on,
    hci_au_on,
    hci_fa_on,
    hci_fb_on,
    hci_fp_on,
    hci_ra_on,
    hci_rb_on,
    hci_vo_on,
    hci_rg_on,
    hci_ver_get,
};

int bulk_hci_xact () {
  int idx = 0;
  int num_cmds = sizeof (hci_lens) / sizeof (int);
  int num_cmds2 = sizeof (hci_cmds) / sizeof (int);
  if (num_cmds2 != num_cmds) {
    return (-1);
  }
  for (idx = 0; idx < num_cmds; idx++) {
    logd ("bulk_hci_xact: %d", idx);
    if (hci_xact (hci_cmds [idx], hci_lens [idx]) < 0)
      return (-1);
  }
  return (0);
}
#endif  //#ifdef  FM_MODE


/*
/sys/class/rfkill/:
lrwxrwxrwx root     root              2012-06-19 04:09 rfkill0 -> ../../devices/platform/bt_power.0/rfkill/rfkill0
lrwxrwxrwx root     root              2012-06-19 04:09 rfkill1 -> ../../devices/platform/bluesleep/rfkill/rfkill1
*/

// BT power utilities derived from cm7src/system/bluetooth/bluedroid/bluetooth.c :

/*char * */ void rfkill_state_file_get (char * rfkill_state_file, size_t rsf_size, char * type) {       // !!!! CM9 has rfkill for bcm4329 AND hci0 !!!!

  if (rsf_size < 1)
    return;// (NULL);

  if (rfkill_state_file [0])
    return;// (rfkill_state_file);                                      // Return cached value if already set...

  char path[64] = {0};
  char buf [64] = {0};
  int fd = -1, ret = 0, id = 0;

  for (id = 0; id < 256; id ++) {                                       // For all values of id (0-255) that have a type file...

    snprintf (path, sizeof (path), "/sys/class/rfkill/rfkill%d/type", id);
    //permissions_set (path, "666");                                      // Ensure permissions to open type file. Does this work in app ? Get EPERM ?

    fd = open (path, O_RDONLY);                                         // Open type file
    if (fd < 0) {
      logd ("rfkill_state_file_get open %s errno: %s (%d)", path, strerror (errno), errno);   // Normal error
      return;// (NULL);
    }

    ret = read (fd, & buf, sizeof (buf));                               // Get contents of type file
    close (fd);

    logd ("rfkill_state_file_get for path: \"%s\"  read: \"%s\" .", path, buf );

    if (ret >= strlen (type) && memcmp (buf, type, strlen (type)) == 0){// If type starts with variable tpe, eg "bluetooth" or "fm"

      if (rfkill_state_file [0] == 0) {                                 // If not set yet, IE if first found...
        snprintf (rfkill_state_file, rsf_size, "/sys/class/rfkill/rfkill%d/state", id);
        //permissions_set (rfkill_state_file, "666");                       // Ensure permissions to open rfkill file. Does this work in app ? Get EPERM ?
      }

      //return;// (rfkill_state_file);
    }
  }
  return;// (NULL);
}

int rfkill_state_get (char * rfkill_state_file, size_t rsf_size, char * type) {     // !!!! CM9 has rfkill for bcm4329 AND hci0 !!!!
  int fd = -1, ret = -1;
  char rbuf = 0;

  rfkill_state_file_get (rfkill_state_file, rsf_size, type);            // Get RFKill state file, eg: "bluetooth" or "fm"
  if (rfkill_state_file [0] == 0)                                       // If no state file of type desired found...
    return (-1);

  fd = open (rfkill_state_file, O_RDONLY);                              // Open state file
  if (fd < 0) {
    loge ("rfkill_state_get open \"%s\" errno: %s (%d)", rfkill_state_file, strerror (errno), errno);
    return (-2);
  }
  ret = read (fd, &rbuf, 1);                                            // Read 1 byte
  if (ret < 1) {                                                        // If don't have at least 1 byte...
    loge ("rfkill_state_get read \"%s\" ret: %d  errno: %s (%d)", rfkill_state_file, ret, strerror (errno), errno);
    close (fd);
    return (-3);
  }
  close (fd);

  if (rbuf == '1')
    return (1);

  else if (rbuf == '0')
    return (0);
 
  return (-4);
}


int rfkill_state_set (int on, char * rfkill_state_file, size_t rsf_size, char * type) {
  int fd = -1, ret = -1;

  rfkill_state_file_get (rfkill_state_file, rsf_size, type);            // Get RFKill state file, eg: "bluetooth" or "fm"
  if (rfkill_state_file [0] == 0)
    return (-1);                                                        // Abort error if we can't get state file name

  //permissions_set (rfkill_state_file, "666");                           // Works in SPRTD daemon but doesn't work in app; get EPERM trying to write; Don't need here

  char rfk_cmd [DEF_BUF] = "echo ";                                      // Works in SPRTD daemon but writing directly doesn't work in app; get EPERM even when permissions set to 666
  if (on)
    strlcat (rfk_cmd, "1 > ", sizeof (rfk_cmd));
  else
    strlcat (rfk_cmd, "0 > ", sizeof (rfk_cmd));

  strlcat (rfk_cmd, rfkill_state_file , sizeof (rfk_cmd));

  su_run (rfk_cmd, 0);                                                  // Run the shell command as SU
  su_run (rfk_cmd, 0);                                                  // Repeat to ensure
  return (0);
}


typedef struct {    // BD Address
    __u8 b[6];
} __attribute__((packed)) bdaddr_t;
#ifdef  NONEED
#define BDADDR_ANY   (&(bdaddr_t) {{0, 0, 0, 0, 0, 0}})
#define BDADDR_LOCAL (&(bdaddr_t) {{0, 0, 0, 0xff, 0xff, 0xff}})
#define SOL_L2CAP    6
#define SOL_SCO        17
#define SOL_RFCOMM    18
#define PF_BLUETOOTH AF_BLUETOOTH
#endif

#define SOL_HCI        0
#define BTPROTO_HCI     1

#include "inc/hci.h"
#include "inc/hci_lib.h"

    
int ba2str_size (const bdaddr_t *btaddr, char *straddr, int straddr_size) {
  unsigned char *b = (unsigned char *)btaddr;
  return snprintf (straddr, straddr_size, "%2.2X:%2.2X:%2.2X:%2.2X:%2.2X:%2.2X",b[7], b[6], b[5], b[4], b[3], b[2]);
}
int str2ba(const char *str, bdaddr_t *ba) {
  int i= 0;
  for (i = 5; i >= 0; i--) {
    ba->b[i] = (uint8_t) strtoul (str, (char **)&str, 16);
    str++;
  }
  return 0;
}

//#define DISP_INFO
#ifdef DISP_INFO
  struct hci_dev_info di = {dev_id: dev_id};
  char addr[18] = "abc";

  ret= ioctl (btsock, HCIGETDEVINFO, (void*) &di);
  logd (" btsock: %d",btsock);
  if (!ret) {
    ba2str_size (&di.bdaddr, addr, sizeof (addr));
    logd (" HCIGETDEVINFO ioctl name: %s  addr: %s",di.name,addr); // !!!! Crashes here when BT is off
  }
  else {
    loge (" HCIGETDEVINFO ioctl ret: %d  errno: %s", ret, errno);
  }
#endif

int tmo_write (int fd, char * buf, int buf_len, int tmo_ms) {
  //logd ("tmo_write buf_len: %d  tmo_ms: %d", buf_len, tmo_ms);
  int buf_left = buf_len, buf_sent = 0;
  int tmo_time = ms_get () + tmo_ms;
  while (buf_left > 0) {
    if (ms_get () >= tmo_time) {
      loge ("tmo_write timeout reached of %d milliseconds", tmo_ms);
      return (buf_sent);
    }
    int wret = write (fd, & buf [buf_sent], buf_left);
    if (wret < 0) {
      if (errno == EAGAIN || errno == EWOULDBLOCK) {
        logd ("tmo_write waiting errno: %s (%d)", strerror (errno), errno);
        ms_sleep (10);                                                  // Wait 10 milliseconds
        continue;
      }
      loge ("tmo_write write errno: %s (%d)", strerror (errno), errno);
      return (buf_sent);
    }
    buf_left -= wret;
    buf_sent += wret;
    if (wret == 0) {                                                    // If wrote 0 bytes (but no error)
      logd ("tmo_write wrote 0 errno: %s (%d)", strerror (errno), errno);
      ////ms_sleep (10);                                                   // USED TO Treat like waiting...: Wait 10 milliseconds
      //ms_sleep (100);                                                  // 100 ms
      //continue;
      return (buf_sent);    // Just return now
    }
    else if (buf_left > 0) {
      logd ("tmo_write write partial buf_len: %d  written: %d", buf_len, wret);
    }
  }
  buf_sent = buf_len - buf_left;
  return (buf_sent);
}

int tmo_read_tmo_error_times = 0;

int tmo_read (int fd, char *buf, int buf_len, int tmo_ms, int single_read) {
  //logd ("tmo_read buf_len: %d  tmo_ms: %d", buf_len, tmo_ms);
  int partial = 0, rret = 0;
  int buf_left = buf_len;                                               // Buffer left = count to read
  int buf_recv = 0;                                                     // Bytes read = 0

  if (tmo_read_tmo_error_times >= 10) {                                 // If excessive errors...
    if (tmo_read_tmo_error_times ++ < 1000)                             // Mark as another error. If under 1000....
      return (0);                                                       // Return as if no error, but 0 bytes read
    tmo_read_tmo_error_times = 0;                                       // Reset after 1000 times
  }

  int tmo_time = ms_get () + tmo_ms;                                    // Time to timeout

  while (buf_left > 0) {                                                // While we still have bytes to read
    if (ms_get () >= tmo_time) {
      if (tmo_ms != 1) {                                                // Suppress for UART flush
        tmo_read_tmo_error_times ++;                                    // Another timeout error
        logd ("tmo_read timeout reached of %d milliseconds", tmo_ms);
      }
      return (buf_recv);                                                // If timeout,... return number bytes read so far
    }
    tmo_read_tmo_error_times = 0;                                       // Reset errors
    rret = read (fd, & buf [buf_recv], buf_left);                         // Try to read up to number of bytes left to read
    if (rret < 0) {                                                     // If error
      if (errno == EAGAIN || errno == EWOULDBLOCK) {                      // If would block
        //logd ("tmo_read waiting errno: %s (%d)", strerror (errno), errno);
        ms_sleep (1);
        continue;                                                       // Sleep a bit and continue loop
      }
      loge ("tmo_read read errno: %s (%d)", strerror (errno), errno);   // If other error...
      return (buf_recv);                                                // Return number bytes read so far
    }
    buf_left -= rret;                                                   // If bytes were read,... left reduced by number read this loop
    buf_recv += rret;                                                   // Received increased by number read this loop
    if (single_read)
        //!!!! For BT just return; if buf was big enough we have all we nead; a 2nd read never returns more data
      return (buf_recv);                                                // Return number bytes read (should be all of them)

    if (buf_left > 0) {                                                 // If still bytes to get,... display debug message
      logd ("tmo_read read partial buf_len: %d  read: %d", buf_len, rret);
      partial = 1;                                                      // Flag as partial read
    }
    
  }
  if (partial && buf_len == 3 && buf_recv == 3) {                       // To attempt to fix extra byte response to reset_start ()...
    if (buf [0] != 4 && buf [1] == 4 && buf [2] == 0x0e) {
      loge ("tmo_read removing bogus byte: 0x%x", buf [0]);
      ms_sleep (1);                                                    // Sleep a bit to ensure next byte
      buf [0] = 4;
      buf [1] = 0x0e;
      //buf [2] =4;
      rret = read (fd, & buf [2], 1);                                    // Read size byte
      if (rret < 0) {                                                   // If error
        loge ("tmo_read removed bogus byte but can't get size byte");
        return (2);
      }
      loge ("tmo_read added size byte: 0x%x", buf [2]);
      return (3);
    }
  }
  return (buf_recv);                                                    // Return number bytes read (should be all of them)
}

int btsock_get () {
  int ret = 0;
  int btsock = socket (AF_BLUETOOTH, SOCK_RAW, BTPROTO_HCI);            // EPERM if not root
  if (btsock < 0) {
    loge ("btsock_get socket errno: %d", errno);
    return (-3);
  }
  //logd ("btsock_get btsock: %d",btsock);
  int opt = 1;
  if (setsockopt (btsock, SOL_HCI, HCI_DATA_DIR, & opt, sizeof (opt)) < 0) {
    loge ("btsock_get setsockopt errno: %d", errno);
    //close (btsock);                                                   // Ignore error; recoverable ?
    //btsock = -1;
    //return (-1);
  }
  struct sockaddr_hci haddr = {0};                                      // Bind socket to the HCI device
  haddr.hci_family = AF_BLUETOOTH;
  haddr.hci_dev = 0;  // hci0
  if (bind (btsock, (struct sockaddr *) &haddr, sizeof (haddr)) < 0) {
    loge ("btsock_get bind errno: %s (%d)", strerror (errno), errno);   // No such device (19) on HTC One
    if (errno == 19) {
      close (btsock);                                                   // Was "Ignore error; recoverable ?"    Bluetooth off gets 19 = No Such Device
      btsock = -1;
      return (-1);
    }
  }
  struct hci_filter flt = {0};                                          // Setup event filter. This is required when using read ()
  hci_filter_clear(&flt);
  hci_filter_set_ptype(HCI_EVENT_PKT, &flt);
  hci_filter_all_events (&flt);
  ret= setsockopt (btsock, SOL_HCI, HCI_FILTER, &flt, sizeof (flt));
  if ( ret < 0) {
    loge ("btsock_get setsockopt HCI filter errno: %d", errno);
    close (btsock);
    btsock = -1;
    return (-4);
  }

  ret = noblock_set (btsock);                                           // ? Better than setsockopt  SO_RCVTIMEO  SO_SNDTIMEO
  if ( ret < 0) {
    loge ("btsock_get noblock_set error ret: %d", ret);
    close (btsock);
    btsock = -1;
    return (-5);
  }
  return (btsock);
}


// Don't need noblock_set as uart_fd was opened with O_NONBLOCK
int uart_send (unsigned char * buf, int len) {
                                                                        // First do a non-blocking read to remove any previous stuff
  int rret = read (uart_fd, hci_recv_buf, sizeof (hci_recv_buf));
  if (rret > 0) {
    loge ("uart_send rret: %d", rret);
    hex_dump ("", 32, hci_recv_buf, rret);
    if (rret >= 4) {                                                    // !! 7 is a sign of correct baudrate; sometimes get 2 or 4
      //disable_baudrate_next = 1;
    }
  }
  //logd ("uart_send bytes: %d", len);
  //if (len < 128)
  //  hex_dump ("", 32, buf, len);                                      // Only dump smaller packets
  int wret = tmo_write (uart_fd, & buf [4], len - 4, 800);//1000);      // Write HCI command, waiting up to 0.8 (1) second to complete
  if (wret != len - 4) {
    loge ("uart_send wret: %d", wret);
    return (-1);
  }
  return (0);
}

int hcia_mode = 0;                                                      // HCI Access mode: 1 = BT sockets, 2 = UART, 3 = tifm
int btsock = -1;
int btsock_get_error_times = 0;

typedef void * TRANSAC;
TRANSAC bfm_send (char * buf, int len);
static int   hcib_dealloc_mem_cb       (TRANSAC transac, char *p_buf);

int bluedroid_cmd (char * cmd, int cmd_len) {                           // Do bluedroid mode command

//Tx:
//if (cmd [4] == 0x01

  uint16_t ocf = cmd [5];                                               // 0x15
  uint16_t ocf_hi = (cmd [6] & 0x03) << 8;
  ocf |= ocf_hi;
  uint16_t ogf = (cmd [6] & 0xfc) >> 2;

  char * hci_data = & cmd [8];
  int hci_len = cmd_len - 8; // cmd [7]

  logd ("bluedroid_cmd ogf: 0x%x  ocf: 0x%x  hci_data: %p  hci_len: %d", ogf, ocf, hci_data, hci_len);

#ifdef  HCI_BLUEDROID
  bfm_rx_len = 0;

  TRANSAC tx_mem = bfm_send (& cmd [5], hci_len + 3);
//#endif
//tifm_cmd
//uart_cmd

  int tmo_ctr = 0;
  while (tx_mem && tmo_ctr ++ < 2000) {   // 2 seconds

    if (bfm_rx_len) {
      if (bfm_rx_len > 0 && bfm_rx_len < 288) {
        hci_recv_buf [0] = 0;
        hci_recv_buf [1] = 1;
        memcpy (& hci_recv_buf [2], bfm_rx_buf, bfm_rx_len);
      }
      //if (tx_mem)
      //  hcib_dealloc_mem_cb (tx_mem, (char *) & ((char *)tx_mem) [8]);
      int rret1 = bfm_rx_len - 6;
      int rret2 = bfm_rx_buf [1] + 4;
      if (rret1 != rret2)
        loge ("bluedroid_cmd rret1: %d  rret2: %d", rret1, rret2);
      return (rret1);
    }
    usleep (1000);
  }

  //if (tx_mem)
  //  hcib_dealloc_mem_cb (tx_mem, (char *) & ((char *)tx_mem) [8]);

  if (tmo_ctr >= 2000)
    return (-1);
#endif

  return (-2);
}

int sock_cmd (char * cmd, int cmd_len) {                                // Do BT Sockets mode command
  int dev_id = -1, ret = 0;
  cmd_len -= 4;                                                         // Adjust length to that of actual HCI command
  cmd += 4;                                                             // Point to actual HCI command

  hci_recv_buf [0] = 0xff;                                              // Default return = error

  if (btsock < 0) {                                                     // If no cached BT socker...
    if (btsock_get_error_times >= 10) {                                 // If excessive errors...
      if (btsock_get_error_times ++ < 1000)                             // Mark as another error. If under 1000....
        return (-1);
      btsock_get_error_times = 0;                                       // Reset after 1000 times
    }
    btsock = btsock_get ();
    if (btsock < 0) {                                                   // Error already logged
      btsock_get_error_times ++;                                        // Another error
      return (-1);
    }
  }  
  btsock_get_error_times = 0;                                           // Reset errors

  int wr_tmo = 800;//1000;
  int rd_tmo = 800;//1000;
//unsigned char hci_reset[] =             { 0, 0, 0, 0, 0x01, 0x03, 0x0c, 0x00 };                                     // OGF:    3    OCF:    3   (Host Controller & Baseband Commands, Reset)
  if (cmd [4] == 0x01 && cmd [5] == 0x03 && cmd [6] == 0x0c) {   // If hci_reset
    wr_tmo = 800;//1000;
    rd_tmo = 3000;//5000;
  }
  int wret = tmo_write (btsock, cmd, cmd_len, wr_tmo);                  // Write HCI command, waiting up to wr_tmo milliseconds
  if (wret != cmd_len) {
    loge ("sock_cmd tmo_write error cmd_len: %d  written: %d", cmd_len, wret);
    close (btsock);
    btsock = -1;
    return (-1);
  }
                                                                        // Read HCI event in response, waiting up to 0.8/3 seconds, doing a single read as needed for BT sockets
  int rret = tmo_read (btsock, & hci_recv_buf [1], sizeof (hci_recv_buf) - 1, rd_tmo, 1);
  if (rret < 3) {                                                       // If an error or less than the minimum of 3 bytes read...
    loge ("sock_cmd tmo_read read: %d", rret);
    close (btsock);
    btsock = -1;
    return (-1);
  }
  if (rret != hci_recv_buf [3] + 3) {                                   // If read length doesn't match the expected length...
    loge ("sock_cmd tmo_read read: %d  expected: %d", rret, hci_recv_buf [3] + 3);
    close (btsock);
    btsock = -1;
    return (-1);
  }
  if (hci_recv_buf [7]) {                                                // Get 12 = "Command Disallowed" if hcitool run even once
    loge ("sock_cmd hci error: %d %s", hci_recv_buf [7], hci_err_get (hci_recv_buf [7]));
    close (btsock);
    btsock = -1;
    return (rret + 1);    //-1);  // Let the app deal with the HCI error
  }

/*// buf [1] = 1 for HCI Event       ?? 4 on serial port ?
  // buf [2] = Event Code: Should be 0x0e (Command Complete)
  int read_remain= (int) hci_recv_buf [3];                               // Set remaining length to read        "Parameter Total Length"
  // buf [4]         Num_HCI_Command_Packets (1 on BC)
  // buf [5],buf [6]  Command_Opcode (That caused this event)
  // buf [7]         HCI Error code (or length on Tx)
  // buf [8]...      Return_Parameter(s) (Optional)
  logd ("sock_cmd read_remain: %d", read_remain);
  hex_dump (" ", 32, hci_recv_buf, 4);
  rret = tmo_read (btsock, &hci_recv_buf [4], 1, 5000);//read_remain, 5000);
  if (rret != read_remain) {
    loge ("sock_cmd read2 read: %d", rret);
    close (btsock);
    btsock = -1;
    return (-6);
   }*/


    close (btsock);
    btsock = -1;    // ?? To fix interference ??


  hci_recv_buf [0] = 0;                                                  // Success
  //logd ("sock_cmd read: %d",rret+1);
  //hex_dump (" ", 32, hci_recv_buf,rret+1);
  return (rret + 1);                                                    // Return size in hci_recv_buf
}

int uart_recv (int fd, unsigned char * buf, int flush) {
  int rret = 0;
  if (flush)
    rret = tmo_read (fd, & buf [1], 3,   1, 0);                         // W/ timeout 1 ms, Read first 3 bytes, doing multiple reads if needed
  else
    //rret = tmo_read (fd, & buf [1], 3, 5000, 0);                      // W/ timeout 5 s, Read first 3 bytes, doing multiple reads if needed
    rret = tmo_read (fd, & buf [1], 3, 800, 0);                         // W/ timeout 0.8 s, Read first 3 bytes, doing multiple reads if needed
  if (rret != 3) {                                                      // If a read error or 3 bytes not read
    if (! flush)
      loge ("uart_recv error 1 rret: %d  flush: %d", rret, flush);
    return (-1);
  }
                                                                        // Else 3 bytes read OK...
  // buf [1] = 1 for HCI Event
  // buf [2] = Event Code: Should be 0x0e (Command Complete)
  int read_remain = (0xff & buf [3]);                                   // Set remaining length to read        "Parameter Total Length"
  // buf [4]         Num_HCI_Command_Packets (1 on BC)
  // buf [5],buf [6]  Command_Opcode (That caused this event)
  // buf [7]         HCI Error code (or length on Tx)
  // buf [8]...      Return_Parameter(s) (Optional)
      
  rret = tmo_read (fd, & buf [4], read_remain, 800, 0);                 // W/ timeout 0.8 (1) s, Read remaining bytes, doing multiple reads if needed
  if (rret != read_remain) {                                            // If read error or partial read...
    loge ("uart_recv error 2 rret %d  read_remain %d  flush: %d", rret, read_remain, flush);
    return (-1);
  }
  if (flush) {
    logd ("uart_recv flushed bytes: %d", rret + 3);
    //if (rret < 128)                                                   // Only dump smaller packets
    hex_dump ("", 32, & buf [0],rret + 4);
  }
  return (rret + 4);                                                    // Return positive total length of normalized standard HCI response packet
}


int uart_cmd (char * cmd, int cmd_len) {                                // Do UART mode command
  int ctr = 0;
  int fret = 0;
  for (ctr = 0; ctr < 128 && fret != -1; ctr ++) {
    fret = uart_recv (uart_fd, hci_recv_buf, 1);                        // Flush Receive response event via UART
    if (fret != -1)
      logd ("uart_cmd uart_recv fret: %d  flushed: 0x%x", fret, hci_recv_buf [4]);
  }
  int wret = uart_send (cmd, cmd_len);                                  // Send command via UART
  if (wret) {
    loge ("uart_cmd uart_send error wret: %d", wret);
    return (-1);
  }
  int rret = uart_recv (uart_fd, hci_recv_buf, 0);                      // Receive response event via UART, no flush
  return (rret);
}


/*
#define u8  uint8_t
#define u16 uint16_t
#define u32 uint32_t
*/
//int fmc_send_cmd (struct fmdev *fmdev, u8 fm_op, u16 type, void *payload, unsigned int payload_len, void *response, int *response_len);
//u32 fm_send_cmd  (struct fmdev *fmdev, u8 fm_op, u16 type, void *payload,          int payload_len, struct completion * wait_completion);

/*strings libfmchr.so |less
http://c-cpp.r3dcode.com/files/linux-kernel/3/2-rc2/drivers/media/radio/wl128x/fmdrv_common.c

struct fmdev m_fmdev;
u8 fm_op = 0;
u16 type = 0;
void payload [1024] = {0};
unsigned int payload_len = 0;
char response [1024] = {0};
int response_len = 0;

int ret = fmc_send_cmd (& m_fmdev, fm_op, type, & payload, payload_len, & response, & response_len);

struct completion wait_completion = {0};
u32 uret = fm_send_cmd (& m_fmdev, fm_op, type, & payload, payload_len, & wait_completion);

fm_send_req

*/


// http://git.omapzoom.org/?p=platform/hardware/ti/wpan.git;a=blob;f=fmradio/fm_chrlib/fm_chrlib.c;h=b462e5ed50a7bf7f62f55dce141d162076664f84;hb=gingerbread

// less /a/Documents/cell/ti/wl128x/fmdrv.h
/*
int fm_send_cmd (int dd, uint16_t ogf, uint16_t ocf, uint8_t plen, void * param);
int fm_send_req (unsigned short hci_op, unsigned char * cmd_params, unsigned short params_len, unsigned char * user_data);
int fm_open_dev (int dev_id, void * fm_interrupt_cb, void * cmd_cmplt_cb, void * hHandle);
int fm_close_dev (int dd);
*/

//#define USE_TIFM_LIB
#ifdef  USE_TIFM_LIB
#include <dlfcn.h>                                                      // !!!! ioctl() method may work on Intel based Razr I
int no_libfmchr = 0;
void * libfmchr_handle = NULL;
int (* fm_open_dev) (int dev_id, void * fm_interrupt_cb, void * cmd_cmplt_cb, void * hHandle);
int (* fm_close_dev) (int dd);
int (* fm_send_req) (unsigned short hci_op, unsigned char * cmd_params, unsigned short params_len, unsigned char * user_data);
    //int (* fm_send_cmd) (int dd, uint16_t ogf, uint16_t ocf, uint8_t plen, void * param);

int libfmchr_setup () {
  if (no_libfmchr)
    return (-1);

  if (libfmchr_handle == NULL) {
    no_libfmchr = 1;
    libfmchr_handle = dlopen ("libfmchr.so", RTLD_LAZY);
    if (libfmchr_handle == NULL)
      return (-1);
    //if ((fm_send_cmd = dlsym (libfmchr_handle, "fm_send_cmd")) == NULL)
    //  return (-1);
    if ((fm_send_req = dlsym (libfmchr_handle, "fm_send_req")) == NULL)
      return (-1);
    if ((fm_open_dev = dlsym (libfmchr_handle, "fm_open_dev")) == NULL)
      return (-1);
    if ((fm_close_dev = dlsym (libfmchr_handle, "fm_close_dev")) == NULL)
      return (-1);
    no_libfmchr = 0;
    return (0);
  }
  return (0);
}

#else
#endif

int tifm_resp = 0;
int tifm_resp_size = 0;

//int tifm_reg = 0;
/*
int tifm_pwr_off () {
  int ret = 0;
  uint16_t ocf = 0x137; // hci_op
  uint8_t  hci_len = 2;
  unsigned char hci_data [1024] = {1, 0, 0, 0, 0};
  unsigned char hci_resp [1024] = {0};
  tifm_resp = 0;
  ret = fm_send_req (ocf, hci_data, hci_len, hci_resp);
  //logd ("tifm fm_send_req: %x %x %x %x     %x %x %x %x", hci_resp[0], hci_resp[1], hci_resp[2], hci_resp[3], hci_resp[4], hci_resp[5], hci_resp[6], hci_resp[7]);
  if (ret < 0) {
    loge ("tifm fm_send_req error: %d", errno);
    return (-1);
  }
  int tmo_ctr = 0;
  while (tifm_resp == 0 && tmo_ctr ++ < 1000)
    ms_sleep (1);
  if (tmo_ctr < 1000)
    return (0);
  return (-1);
}

int tifm_pwr_on () {
  int ret = 0;
  uint16_t ocf = 0x137; // hci_op
  uint8_t  hci_len = 6;//2;
  unsigned char hci_data [1024] = {1, 1, 0, 0, 0};
  unsigned char hci_resp [1024] = {0};
  tifm_resp = 0;
  ret = fm_send_req (ocf, hci_data, hci_len, hci_resp);
  //logd ("tifm fm_send_req: %x %x %x %x     %x %x %x %x", hci_resp[0], hci_resp[1], hci_resp[2], hci_resp[3], hci_resp[4], hci_resp[5], hci_resp[6], hci_resp[7]);
  if (ret < 0) {
    loge ("tifm fm_send_req error: %d", errno);
    return (-1);
  }
  int tmo_ctr = 0;
  while (tifm_resp == 0 && tmo_ctr ++ < 1000)
    ms_sleep (1);
  if (tmo_ctr < 1000)
    return (0);
  return (-1);
}

int tifm_reg_set (int reg, int value) {
  int ret = 0;
  uint16_t ocf = 0x135; // hci_op
  uint8_t  hci_len = 5;
  unsigned char hci_data [1024] = {0x0a, 2, 0, 0, 0x94};
  hci_data [0] = reg;
  hci_data [3] = value / 256;
  hci_data [4] = value % 256;
  unsigned char hci_resp [1024] = {0};
  tifm_resp = 0;
  ret = fm_send_req (ocf, hci_data, hci_len, hci_resp);
  //logd ("tifm fm_send_req: %x %x %x %x     %x %x %x %x", hci_resp[0], hci_resp[1], hci_resp[2], hci_resp[3], hci_resp[4], hci_resp[5], hci_resp[6], hci_resp[7]);
  if (ret < 0) {
    loge ("tifm fm_send_req error: %d", errno);
    return (-1);
  }
  int tmo_ctr = 0;
  while (tifm_resp == 0 && tmo_ctr ++ < 1000)
    ms_sleep (1);
  if (tmo_ctr < 1000)
    return (0);
  return (-1);
}

int tifm_reg_get (int reg) {
  int ret = 0;
  uint16_t ocf = 0x133; // hci_op
  uint8_t  hci_len = 3;//3;
  //unsigned char hci_data [1024] = {0x00, 0x80, 0};
  unsigned char hci_data [1024] = {0x2b, 0x02, 0};
  hci_data [0] = reg;
  unsigned char hci_resp [1024] = {0};
  tifm_resp = 0;
  ret = fm_send_req (ocf, hci_data, hci_len, hci_resp);
  //logd ("tifm fm_send_req: %x %x %x %x     %x %x %x %x", hci_resp[0], hci_resp[1], hci_resp[2], hci_resp[3], hci_resp[4], hci_resp[5], hci_resp[6], hci_resp[7]);
  if (ret < 0) {
    loge ("tifm fm_send_req error: %d", errno);
    return (-1);
  }
  int tmo_ctr = 0;
  while (tifm_resp == 0 && tmo_ctr ++ < 1000)
    ms_sleep (1);
  if (tmo_ctr < 1000)
    return (tifm_reg);//0);
  return (-1);
}

int ti_freq_lo;// = 87500;

int tifm_freq_set (int freq) {        // Maximum frequency KHz = (65535 * 50) + 76000/87500 = 3352.75 Mhz / 3364.25
  tifm_reg_set (0x2d, 0);                           // Cancel any previous search in progress
  tifm_reg_set (0x0a, (freq - ti_freq_lo) / 50);
  tifm_reg_set (0x2d, 1);                           // Preset
  return (0);
}
*/

#ifdef  USE_TIFM_LIB

void fm_interrupt_cb () {
  logd ("tifm fm_interrupt_cb");
  return;
}
typedef struct
{
    int                     eResult;
    unsigned char           uEvtParamLen;
    unsigned char           *pEvtParams;
} TCmdComplEvent;


    //hci_recv_buf [0] = 0;
    //return (tifm_resp_size + 8);

void cmd_cmplt_cb (void * v1, TCmdComplEvent * CmdComplEvent) {
  //logd ("tifm cmd_cmplt_cb v1: %p  CmdComplEvent: %p", v1, CmdComplEvent);

  //int * cv1 = (int *) v1;
  //if (cv1 != NULL)
  //  logd ("tifm cmd_cmplt_cb: %p %d %d %d %p", v1, *cv1, CmdComplEvent->eResult, CmdComplEvent->uEvtParamLen, CmdComplEvent->pEvtParams);
  //else
  //  logd ("tifm cmd_cmplt_cb: %p %d %d %p", v1, CmdComplEvent->eResult, CmdComplEvent->uEvtParamLen, CmdComplEvent->pEvtParams);

  int ctr = 0;
  if (CmdComplEvent->uEvtParamLen >= 0 && CmdComplEvent->uEvtParamLen < 270) {
//    if (CmdComplEvent->uEvtParamLen == 2) {
//      tifm_reg = CmdComplEvent->pEvtParams [0] * 256 + CmdComplEvent->pEvtParams [1];
//      //logd ("tifm reg: %x", tifm_reg);
//    }
    //else {
    //  for (ctr = 0; ctr < CmdComplEvent->uEvtParamLen ; ctr ++) {
    //    logd ("tifm resp: %x", CmdComplEvent->pEvtParams [ctr]);
    //  }
    //}
  }

  tifm_resp_size = CmdComplEvent->uEvtParamLen;
  if (tifm_resp_size >= 0 && tifm_resp_size < 270) {
    hci_recv_buf [7] = (char) tifm_resp_size;
    memcpy (& hci_recv_buf [8], CmdComplEvent->pEvtParams, tifm_resp_size);
  }
  tifm_resp = 1;
  return;
}

int mr_tifm_sig = 0x12345678;
#endif

int fm_fd = -1;
int tifm_start () {
#ifdef  USE_TIFM_LIB
  if (libfmchr_setup ()) {
    loge ("tifm tifm_start libfmchr_setup error");
    return (-1);
  }
  fm_fd = fm_open_dev (0, (void *) fm_interrupt_cb, (void *) cmd_cmplt_cb, & mr_tifm_sig);
#else
  fm_fd = open ("/dev/tifm", O_RDWR);

  int oflags = fcntl (fm_fd, F_GETFL);
  fcntl (fm_fd, F_SETOWN, getpid());
  int ret = fcntl (fm_fd, F_SETFL, oflags | FASYNC);
  if (ret < 0) {
    loge ("tifm_start fcntl error: %d", errno);
    return (-1);
  }
#endif
  if (fm_fd < 0) {
    loge ("tifm_start /dev/tifm open error: %d", errno);
    return (-1);
  }
  logd ("tifm_start fm_fd: %d", fm_fd);

  ms_sleep (1);
/*
  tifm_pwr_on ();

  tifm_reg_get (0x2b);
  tifm_reg_get (0x2a);

  tifm_reg_set (0x20, 3);

  tifm_freq_set (88500);

  int ctr = 0;
  for (ctr = 0; ctr < 4; ctr ++) {
    tifm_reg_get (0x01);
  }

  //tifm_pwr_off ();

  //sleep (10);
*/
  return (0);
}

int tifm_stop () {
#ifdef  USE_TIFM_LIB
  int ret = fm_close_dev (fm_fd);
  logd ("tifm fm_close_dev: %d", ret);
  if (ret)
    return (-1);
#else
  if (fm_fd >= 0) {
    if (close (fm_fd) < 0) {
      fm_fd = -1;
      loge ("tifm_stop /dev/tifm close error: %d", errno);
      return (-1);
    }
  }
  fm_fd = -1;
#endif
  return (0);
}

typedef struct {
    uint16_t    opcode;        /* OCF & OGF */
    uint8_t        plen;
} __attribute__ ((packed))    hci_command_hdr;
#define cmd_opcode_pack(ogf, ocf)    (uint16_t)((ocf & 0x03ff)|(ogf << 10))
#define htobs(a)        __cpu_to_le16(a)

typedef struct {
    uint8_t         evt;
    uint8_t         plen;
} __attribute__ ((packed))      hci_event_hdr;

#define EVT_CMD_COMPLETE 		0x0E
typedef struct {
	uint8_t         ncmd;
	uint16_t        opcode;
} __attribute__ ((packed)) evt_cmd_complete;

int tifm_cmd (char * cmd, int cmd_len) {                                // Do TIFM mode command

  //hci_recv_buf [0] = 0xff;                                               // Default return = error
  hci_recv_buf [7] = 0xff;                                               // Default return = error

  if (fm_fd < 0)
    return (-1);

  //cmd_buf [5] = ocf & 0x00ff;
  //cmd_buf [6] = ((ocf & 0x0300) >> 8) | ((ogf & 0x3f) << 2);
  //cmd_buf [7] = cmd_len - 8;

#ifdef  USE_TIFM_LIB
  int ret = 0;
  uint16_t ocf = cmd [5];   //0x133; // hci_op
  uint16_t ocf_hi = (cmd [6] & 0x03) << 8;
  ocf |= ocf_hi;
  tifm_resp = 0;

  char * hci_data = & cmd [8];
  int hci_len = cmd_len - 8; // cmd [7]

  unsigned char hci_resp [1024] = {0};
  ret = fm_send_req (ocf, hci_data, hci_len, hci_resp);
  //logd ("tifm fm_send_req: %x %d", ocf, hci_len);
  if (ret < 0) {
    loge ("tifm_cmd fm_send_req error: %d", errno);
    return (-1);
  }
  int tmo_ctr = 0;
  while (tifm_resp == 0 && tmo_ctr ++ < 1000)
    ms_sleep (1);
  if (tmo_ctr < 1000) {
    hci_recv_buf [0] = 0;
    hci_recv_buf [7] = 0;

    return (tifm_resp_size + 8);
  }
  return (-1);
#else
/*  hci_command_hdr hc;
  int skb_len = 4 + hci_len;
  char skb [skb_len];

  hc.opcode = htobs (cmd_opcode_pack(0x3f, ocf));
  hc.plen = hci_len;

  skb [0] = HCI_COMMAND_PKT;
  memcpy (& skb [1], & hc, HCI_COMMAND_HDR_SIZE);

  memcpy (& skb [4], hci_data, hci_len);
  int ctr = 0;
  for (ctr = 0; ctr < skb_len; ctr ++)
    loge ("%x", skb [ctr]);
  //  printf ("%x ", skb [ctr]);
  //printf ("\n");
  loge ("----");
  for (ctr = 0; ctr < cmd_len; ctr ++)
    loge ("%x", cmd [ctr]);
  loge ("--------");

  while (write (fm_fd, skb, skb_len) < 0) {
*/
  while (write (fm_fd, & cmd [4], cmd_len - 4) < 0) {
    if (errno == EAGAIN || errno == EINTR)
      continue;
    loge ("tifm_cmd write error: %d", errno);
    return (-1);
  }

  unsigned char buf [HCI_MAX_EVENT_SIZE];
  int times = 0, len = 0;
  while (times ++ < 1000) {

    if ((len = read (fm_fd, buf, sizeof (buf))) < 0) {
      if (errno == EAGAIN || errno == EINTR) {
        ms_sleep (1);
        continue;
      }
      if (times % 100 == 1)
        loge ("tifm_cmd read error %d: %d", times, errno);
      ms_sleep (1);
      continue;
    }

    //hci_event_hdr * hdr = (void *) (buf + 1);
    //unsigned char * ptr = buf + 3;
    //len -= 3;

      // Check for the type of event received
    //if (hdr->evt == EVT_CMD_COMPLETE) {
    if (buf [1] == EVT_CMD_COMPLETE) {
      //evt_cmd_complete * cc = (void *) ptr;
//typedef struct {
//    uint8_t         ncmd;
//    uint16_t        opcode;
//} __attribute__ ((packed)) evt_cmd_complete;

      //ptr += 4;
      //len -= 4;
            
      tifm_resp_size = len - 7;
      if (tifm_resp_size >= 0 && tifm_resp_size < 270) {
        hci_recv_buf [0] = 0;
        hci_recv_buf [7] = 0;
        memcpy (& hci_recv_buf [8], buf + 7, tifm_resp_size);
      }
      tifm_resp = 1;
      return (tifm_resp_size + 8);
    }
    else {
      loge ("tifm_cmd not EVT_CMD_COMPLETE: %d", buf [1]);//hdr->evt);
    }
    ms_sleep (1);
  }    //end of while ()
  return (-1);

#endif
}

int hci_xact_error_times = 0;

int hci_xact (char * cmd, int cmd_len) {                                // Do HCI transaction; BT sockets (sprtd) or UART mode (app) or /dev/tifm / libfmchr.so (app?)

  //logd ("tifm hci_xact: %d %d", cmd_len, hcia_mode);
  
  int rret = 0;
  if (hci_dbg) {
    logd ("hci_xact cmd_len: %d", cmd_len);
    hex_dump ("", 32, cmd, cmd_len);
  }
  hci_recv_buf [0] = 0xff;                                              // Default = error

  if (cmd_len < 8 || cmd_len > 270) {                                   // If invalid size...
    loge ("hci_xact error cmd_len: %d", cmd_len);
    return (-1);                                                        // Done w/ error
  }

  if (hci_xact_error_times >= 10) {                                     // If excessive errors... (Aborts most requests if too many errors to prevent hang)
    if (hci_xact_error_times ++ < 1000)                                 // Mark as another error. If under 1000....
      return (-1);                                                      // Done w/ error
    hci_xact_error_times = 0;                                           // Reset after 1000 times
  }

  if (hcia_mode == 0) {                                                 // If Auto/Unset mode...
    rret = -1;
  }
  else if (hcia_mode == 1) {                                            // If BT Sockets mode...
    rret = sock_cmd (cmd, cmd_len);                                     // Do BT Sockets mode transaction
  }
  else if (hcia_mode == 2) {                                            // If UART mode...
    rret = uart_cmd (cmd, cmd_len);                                     // Do UART mode transaction
  }
  else if (hcia_mode == 3) {                                            // If TIFM mode...
    rret = tifm_cmd (cmd, cmd_len);                                     // Do TIFM mode transaction
  }
  else if (hcia_mode == 4) {                                            // If Bluedroid mode...
    rret = bluedroid_cmd (cmd, cmd_len);                                // Do Bluedroid mode transaction
  }

  if (hci_dbg) {
    logd ("hci_xact rret: %d",rret);
    if (rret > 0)
      hex_dump ("", 32, hci_recv_buf, rret);
    else
      hex_dump ("", 32, hci_recv_buf, 16);
  }
  if (rret < 8 || rret > 270) {
    loge ("hci_xact error rret: %d", rret);
    hci_recv_buf [0] = 0xff;                                             // Error
    rret = -1;
    hci_xact_error_times ++;                                            // Another error
  }
  else {
    hci_xact_error_times = 0;                                           // Reset errors
  }
  return (rret);
}

int sock_stop () {
  logd ("sock_stop btsock: %d", btsock);

  if (btsock >= 0)
    close (btsock);
  return (0);
}

int acc_hci_stop () {
  logd ("acc_hci_stop");

  if (hcia_mode == 0)                                                   // If Auto/Unset mode...
    loge ("hcia_mode == 0");
  else if (hcia_mode == 1)                                              // If BT Sockets mode...
    sock_stop ();
  else if (hcia_mode == 2)                                              // If UART mode...
    uart_stop ();
  else if (hcia_mode == 3)                                              // If TIFM mode...
    tifm_stop ();
  else if (hcia_mode == 4)                                              // If Bluedroid mode...
    loge ("hcia_mode == 4 / Bluedroid");

  return (0);
}

char * uart_list [] = {
  "/dev/ttyHS0",                                                        // Most
  "/dev/ttyHS99",
  "/dev/s3c2410_serial0",                                               // Samsung Captivate SGH-i997
  "/dev/ttyHS1",
  "/dev/ttyHS2",
  "/dev/ttyHS3",
  "/dev/ttyHS4",
  "/dev/ttyHS5",
  "/dev/ttyHS6",
  "/dev/ttyHS7",
  "/dev/ttyS1",                                                         // Galaxy Y GT-S5360
  "/dev/ttyS2",
  "/dev/ttyS3",
  "/dev/ttyS4",
  "/dev/ttyS5",
  "/dev/ttyS6",
  "/dev/ttyS7",
  "/dev/ttyS0",
};  
char uart_buf [DEF_BUF] = {0};

#define AID_BLUETOOTH     1002  /* bluetooth subsystem */
char * uart_get () {
  int ctr = 0;
  char * uart = uart_buf;                                               // /dev/ttyHS0...
  char * uart_check = NULL;

  for (ctr = 0; ctr < (sizeof (uart_list) / sizeof (char *)); ctr ++) {
    uart = uart_list [ctr];
    if (file_get (uart)) {                                              // If we have this UART device...
      logd ("uart_get have possible UART: %s", uart);

      uart_check = user_char_dev_get (uart, AID_BLUETOOTH);
      if (uart_check != NULL) {
        logd ("uart_get have bluetooth UART: %s", uart_check);
        return (uart);
      }
      else
        logd ("uart_get not  bluetooth UART: %s", uart_check);
    }
    else {
      logd ("uart_get not exist UART: %s", uart);
    }
  }

  uart = user_char_dev_get ("/dev", AID_BLUETOOTH);                         // !! should we look for tty* etc ? : al /dev|grep -i blu
                                                                        // crw-rw---- system   bluetooth  10, 223 2011-11-20 19:22 uinput
                                                                        // crw-rw-rw- bluetooth bluetooth 248,   0 2011-11-21 00:53 ttyHS0
  if (uart) {
    logd ("uart_get found UART via AID_BLUETOOTH: ", uart);
    return (uart);
  }

  for (ctr = 0; ctr < (sizeof (uart_list) / sizeof (char *)); ctr ++) {
    uart = uart_list [ctr];
    if (file_get (uart)) {                                              // If we have this UART device...
      logd ("uart_get have UART: %s", uart);
      return (uart);
    }
    else {
      logd ("uart_get no   UART: %s", uart);
    }
  }

  logd ("uart_get no UART found");
  return (NULL);
}


char bt_rfkill_state_file [DEF_BUF] = {0};                              // "/sys/class/rfkill/rfkill0/state";

int bt_rfkill_state_set (int state) {
  if (rfkill_state_set (state, bt_rfkill_state_file, sizeof (bt_rfkill_state_file), "bluetooth") < 0) { // UART off/on w/ 0/1: If error...
    loge ("bt_rfkill_state_set rfkill_state_set (%d) error rfkill_state_get: %d", state, rfkill_state_get (bt_rfkill_state_file, sizeof (bt_rfkill_state_file), "bluetooth"));
    return (-1);                                                        // Error
  }
  return (0);                                                           // Done OK
}

int uart_stop () {

  int uart_pwr = rfkill_state_get (bt_rfkill_state_file, sizeof (bt_rfkill_state_file), "bluetooth");
  logd ("uart_stop uart_pwr: %d  uart_fd: %d", uart_pwr, uart_fd);

  if (uart_fd >= 0)                                                     // If UART open...
    close (uart_fd);                                                    // Close

  uart_fd = -1;

  if (uart_pwr <= 0) {                                                  // If UART is already off or error...
    loge ("uart_stop BT is off or error; will not stop UART");
    return (-1);                                                        // Done w/ error
  }

  if (bt_rfkill_state_set (0) < 0) {                                    // UART off: If error...
    loge ("uart_stop bt_rfkill_state_set (0) error");
    return (-1);                                                        // Error
  }

  logd ("uart_stop OK rfkill_state_set: %d", rfkill_state_get (bt_rfkill_state_file, sizeof (bt_rfkill_state_file), "bluetooth"));
  return (0);
}

int uart_start () {
  //logd ("uart_start rfkill_state_get: %d", rfkill_state_get (bt_rfkill_state_file, sizeof (bt_rfkill_state_file), "bluetooth"));

  char * uart = uart_get ();                                            // Get UART filename...
  if (! uart)                                                           // If can't get UART filename, abort
    return (-1);

  permissions_set (uart, "677");                                        // Ensure permissions to open UART. Does this work in app ? Get EPERM ?

  int uart_pwr = rfkill_state_get (bt_rfkill_state_file, sizeof (bt_rfkill_state_file), "bluetooth");
  logd ("uart_start uart_pwr: %d  uart_fd: %d", uart_pwr, uart_fd);

  //uart_fd = -1;                                                         // Invalidate any previous UART file handle for if error we don't close invalid handle

  if (uart_pwr > 0) {                                                  // If UART is already on...
    loge ("uart_start BT is on; will not start UART %s due to %s", uart, bt_rfkill_state_file);
//    return (-1);                                                        // Done w/ error
  }

  if (bt_rfkill_state_set (1) < 0) {                                    // UART on: If error...
    loge ("uart_stop bt_rfkill_state_set (0) error");
//    return (-1);                                                        // Error
  }

  //logd ("uart_start rfkill_state_get: %d", rfkill_state_get (bt_rfkill_state_file, sizeof (bt_rfkill_state_file), "bluetooth"));
  //default_bdaddr_set ();                                              // Get default BT address

  if ( (uart_fd = open (uart, O_RDWR | O_NOCTTY | O_NONBLOCK)) == -1)   // Open UART
    loge ("uart_start open uart: %s  errno: %d", uart, errno);
  else
    logd ("uart_start open uart: %s", uart);
  if (uart_fd < 0) {                                                    // If UART can't be opened...
    bt_rfkill_state_set (0);                                            // UART off
    return (-1);                                                        // Done w/ error
  }

// tmo_read timeout reached

  //if (file_get ("/system/etc/firmware/BCM4335B0_002.001.006.0092.0093.hcd"))
  //  loge ("uart_start no uart_init(115200) because have HTC One BCM4335B0...");
  //else
    uart_init (115200);                                                   // Else... Initialize the UART @ 115200

//  uart_init (3000000);

  //uart_init (9600);
  //baudrate_next

  if (reset_start ()) {                                                 // If reset start error...  !!!! Doesn't work; error comes in patchram_set () !!!!
    loge ("uart_start reset_start error @ 115200");
    uart_init (3000000);                                                // Else... Initialize the UART @ 3000000
    //uart_init (24000000);                                                // Else... Initialize the UART @ 3000000
    if (reset_start ()) {                                               // If reset start error...
      loge ("uart_start reset_start error @ 3000000");
      uart_init (9600);                                                 // Else... Initialize the UART @ 9600
      //uart_init (115200);
      if (reset_start ()) {                                             // If reset start error...
        loge ("uart_start reset_start error @ 9600");
        bt_rfkill_state_set (0);                                        // UART off
        return (-1);                                                    // Done w/ error
      }
    }
  }
  if (patchram_set ()) {                                                // If patchram error...
    loge ("uart_start patchram_set error");
/*
    uart_init (9600);                                                 // Else... Initialize the UART @ 9600
    if (reset_start ()) {                                             // If reset start error...
      loge ("uart_start 2 reset_start error @ 9600");
    }
    else {
      if (patchram_set ()) {                                                // If patchram error...
        loge ("uart_start 2 patchram_set error");
      }
      else {
        if (reset_start ()) {                                                 // Another reset prevents some strange issues like no audio or 64 MHz frequency
          loge ("uart_start 2 reset_start 2 error");
        }
      }
      loge ("!!!!!!!!!!!!!! uart_start recovered at 9600 !!!!!!!!!!!!!");
      return 0;
    }
*/
    uart_baudrate_set (start_baudrate);                                 // Restore port to start baudrate
    bt_rfkill_state_set (0);                                            // UART off
    return (-1);                                                        // Done w/ error
  }

  if (reset_start ()) {                                                 // Another reset prevents some strange issues like no audio or 64 MHz frequency
    loge ("uart_start reset_start 2 error");
    //bt_rfkill_state_set (0);                                            // UART off
    //return (-1);                                                  // ?? No UART mode on Dell Streak due to error ?
  }

  return (0);                                                           // Done success
}

    //if (bdaddr_flag)
    //  if (bdaddr_set ())
    //    continue;
    //if (enable_lpm)
    //  if (enable_lpm_set ())
    //    continue;

    //if (enable_hci) {
    //  if (enable_hci_set ())
    //    continue;
    //  std_files_close ();
    //  while (1) {
    //    sleep(UINT_MAX);                                              // Just sleep for HCI mode
    //  }
    //}
//  break;
//  }

                                                                        // Access HCI start: For App Internal (UART only) or Daemon (BT Sock or UART)

int acc_hci_start (int mode) {                                          // mode: 0 or anything else = automatic (via sprtd), 1 = SOCK mode only,
                                                                        // (unused, 2 = UART mode only (direct via fm_hrdw.c), 3 = TIFM mode

  hcia_mode = 0;                                                        // Default 0 = Auto/Unset mode

  if (mode == 3) {                                                      // TIFM mode
/*
    if (! ti_get ()) {                                     // If TI...     !!! MOD !! Sometimes ROMS have both BCM and TI firmware HCD/BTS files... !!!!
                                                                        // If not TI then done...
      loge ("acc_hci_start error no tifm mode for Non TI");
      return (-1);                                                        // Done w/ error
    }*/
    if (tifm_start ()) {
      loge ("acc_hci_start error no tifm mode");
      return (-1);
    }
    hcia_mode = 3;
    logd ("acc_hci_start success tifm mode");
    return (0);
  }

  if (mode == 4) {                                                      // If Bluedroid mode...
    hcia_mode = 4;
    logd ("acc_hci_start success bluedroid mode");
    return (0);
  }

  if (mode != 2) {                                                      // If not UART mode (direct via app), we try BT Sockets mode first...

    hcia_mode = 1;                                                      // Force BT sockets mode to test for BT sockets

    int rret = hci_xact (hci_ver_get, sizeof (hci_ver_get));            // Get HCI Version (Absolute maximum time = 8 seconds; sock_cmd uses 3 seconds for write and 5 for read)

    if (hci_recv_buf [7] == 0 && rret == 16 && hci_recv_buf [8] >= 1) { // If valid response w/ proper length and no HCI error and version >= 1...
      int manuf = (hci_recv_buf [12]) + (256 * hci_recv_buf [13]);
      logd ("acc_hci_start success SOCK mode manuf: %d", manuf);
      hcia_mode = 1;                                                    // BT sockets mode
      return (0);
    }
    else if (hci_recv_buf [7]) {                                         // If error...
      loge ("acc_hci_start HCI error: %d %s", hci_recv_buf [7], hci_err_get (hci_recv_buf [7]));
    }
    if (mode == 1) {                                                    // If only SOCK mode is acceptable...
      hcia_mode = 2;                                                    // UART mode = NOT BT sockets mode
      loge ("acc_hci_start error no SOCK mode");
      return (-1);                                                      // Done w/ error
    }
  }

  if (mode == 1) {                                                      // If UART mode...
    loge ("acc_hci_start error no uart mode for sprtd");
    return (-1);                                                        // Done w/ error
  }

    // If UART mode specified, or fall through when BT sockets mode not available...
    // UART mode allowed by daemon for cases where it might not work from app.

  hcia_mode = 2;                                                        // UART mode

  if (! bc_get ()) { //ti_get ()) {                                     // If TI...     !!! MOD !! Sometimes ROMS have both BCM and TI firmware HCD/BTS files... !!!!
                                                                        // If not Broadcom (Which supports UART mode) then done...
    loge ("acc_hci_start error no uart mode for Non Broadcom"); //TI");
    return (-1);                                                        // Done w/ error
  }

// !! Need to prevent attempt at uart mode if BT is on, but would be best if a crashed app with UART on could restart in UART mode; Need BT detection for that
// !! The UA mode of app leaves UART power on !!

  //if (btsock >= 0) {
  //  loge ("acc_hci_start btsock indicates BT/UART is already on; will not start UART");
  //  return (-1);
  //}
  //if (rfkill_state_get (bt_rfkill_state_file, sizeof (bt_rfkill_state_file), "bluetooth") != 0) {                                         // If UART is on...
  //  loge ("acc_hci_start UART is already on; will not start UART");
  //  return (-1);                                                      // Done w/ error
  //}

  if (pid_get ("btld", 0)) {                                            // If Broadcom BT is running...
    loge ("acc_hci_start btld is running; will not start UART");
    return (-1);                                                        // Done w/ error
  }

  if (uart_start ()) {
    loge ("acc_hci_start error no bt or uart mode");
    //uart_stop (); // !!
    return (-1);
  }
  logd ("acc_hci_start success uart mode");
  return (0);
}
