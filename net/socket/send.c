/****************************************************************************
 * net/socket/send.c
 *
 *   Copyright (C) 2007-2014, 2016-2017 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name NuttX nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <sys/types.h>
#include <sys/socket.h>
#include <errno.h>

#include <nuttx/cancelpt.h>

#include "pkt/pkt.h"
#include "tcp/tcp.h"
#include "udp/udp.h"
#include "sixlowpan/sixlowpan.h"
#include "local/local.h"
#include "socket/socket.h"
#include "usrsock/usrsock.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: psock_send
 *
 * Description:
 *   The send() call may be used only when the socket is in a connected state
 *   (so that the intended recipient is known). The only difference between
 *   send() and write() is the presence of flags. With zero flags parameter,
 *   send() is equivalent to write(). Also, send(sockfd,buf,len,flags) is
 *   equivalent to sendto(sockfd,buf,len,flags,NULL,0).
 *
 * Parameters:
 *   psock    An instance of the internal socket structure.
 *   buf      Data to send
 *   len      Length of data to send
 *   flags    Send flags
 *
 * Returned Value:
 *   On success, returns the number of characters sent.  On  error,
 *   -1 is returned, and errno is set appropriately:
 *
 *   EAGAIN or EWOULDBLOCK
 *     The socket is marked non-blocking and the requested operation
 *     would block.
 *   EBADF
 *     An invalid descriptor was specified.
 *   ECONNRESET
 *     Connection reset by peer.
 *   EDESTADDRREQ
 *     The socket is not connection-mode, and no peer address is set.
 *   EFAULT
 *      An invalid user space address was specified for a parameter.
 *   EINTR
 *      A signal occurred before any data was transmitted.
 *   EINVAL
 *      Invalid argument passed.
 *   EISCONN
 *     The connection-mode socket was connected already but a recipient
 *     was specified. (Now either this error is returned, or the recipient
 *     specification is ignored.)
 *   EMSGSIZE
 *     The socket type requires that message be sent atomically, and the
 *     size of the message to be sent made this impossible.
 *   ENOBUFS
 *     The output queue for a network interface was full. This generally
 *     indicates that the interface has stopped sending, but may be
 *     caused by transient congestion.
 *   ENOMEM
 *     No memory available.
 *   ENOTCONN
 *     The socket is not connected, and no target has been given.
 *   ENOTSOCK
 *     The argument s is not a socket.
 *   EOPNOTSUPP
 *     Some bit in the flags argument is inappropriate for the socket
 *     type.
 *   EPIPE
 *     The local end has been shut down on a connection oriented socket.
 *     In this case the process will also receive a SIGPIPE unless
 *     MSG_NOSIGNAL is set.
 *
 * Assumptions:
 *
 ****************************************************************************/

ssize_t psock_send(FAR struct socket *psock, FAR const void *buf, size_t len,
                   int flags)
{
  ssize_t ret;

  /* Treat as a cancellation point */

  (void)enter_cancellation_point();

  switch (psock->s_type)
    {
#if defined(CONFIG_NET_PKT)
      case SOCK_RAW:
        {
          /* Raw packet send */

          ret = psock_pkt_send(psock, buf, len);
        }
        break;
#endif

#if defined(CONFIG_NET_TCP) || defined(CONFIG_NET_LOCAL_STREAM)
      case SOCK_STREAM:
        {
#ifdef CONFIG_NET_LOCAL_STREAM
#ifdef CONFIG_NET_TCP
          if (psock->s_domain == PF_LOCAL)
#endif
            {
              /* Local TCP packet send */

              ret = psock_local_send(psock, buf, len, flags);
            }
#endif /* CONFIG_NET_LOCAL_STREAM */

#ifdef CONFIG_NET_TCP
#ifdef CONFIG_NET_LOCAL_STREAM
          else
#endif
            {
#ifdef CONFIG_NET_6LOWPAN
              /* Try 6LoWPAN TCP packet send */

              ret = psock_6lowpan_tcp_send(psock, buf, len);

#if defined(CONFIG_NETDEV_MULTINIC) && defined(NET_TCP_HAVE_STACK)
              if (ret < 0)
                {
                  /* TCP/IP packet send */

                  ret = psock_tcp_send(psock, buf, len);
                }
#endif /* CONFIG_NETDEV_MULTINIC && NET_TCP_HAVE_STACK */
#elif defined(NET_TCP_HAVE_STACK)
              ret = psock_tcp_send(psock, buf, len);
#else
              ret = -ENOSYS;
#endif /* CONFIG_NET_6LOWPAN */
            }
#endif /* CONFIG_NET_TCP */
        }
        break;
#endif /* CONFIG_NET_TCP || CONFIG_NET_LOCAL_STREAM */

#ifdef CONFIG_NET_UDP
      case SOCK_DGRAM:
        {
#ifdef CONFIG_NET_LOCAL_DGRAM
#ifdef CONFIG_NET_UDP
          if (psock->s_domain == PF_LOCAL)
#endif
            {
              /* Local UDP packet send */
#warning Missing logic
              ret = -ENOSYS;
            }
#endif /* CONFIG_NET_LOCAL_DGRAM */

#ifdef CONFIG_NET_UDP
#ifdef CONFIG_NET_LOCAL_DGRAM
          else
#endif
            {
#if defined(CONFIG_NET_6LOWPAN)
              /* Try 6LoWPAN UDP packet send */

              ret = psock_6lowpan_udp_send(psock, buf, len);

#if defined(CONFIG_NETDEV_MULTINIC) && defined(NET_UDP_HAVE_STACK)
              if (ret < 0)
                {
                  /* UDP/IP packet send */

                  ret = psock_udp_send(psock, buf, len);
                }
#endif /* CONFIG_NETDEV_MULTINIC && NET_UDP_HAVE_STACK */
#elif defined(NET_UDP_HAVE_STACK)
              /* Only UDP/IP packet send */

              ret = psock_udp_send(psock, buf, len);
#else
              ret = -ENOSYS;
#endif /* CONFIG_NET_6LOWPAN */
            }
#endif /* CONFIG_NET_UDP */
        }
        break;
#endif /* CONFIG_NET_UDP */

#ifdef CONFIG_NET_USRSOCK
      case SOCK_USRSOCK_TYPE:
        {
          ret = usrsock_sendto(psock, buf, len, NULL, 0);
        }
        break;
#endif /*CONFIG_NET_USRSOCK*/

      default:
        {
          /* EDESTADDRREQ.  Signifies that the socket is not connection-mode
           * and no peer address is set.
           */

          ret = -EDESTADDRREQ;
        }
        break;
    }

  leave_cancellation_point();
  if (ret < 0)
    {
      set_errno(-ret);
      ret = ERROR;
    }

  return ret;
}

/****************************************************************************
 * Name: send
 *
 * Description:
 *   The send() call may be used only when the socket is in a connected state
 *   (so that the intended recipient is known). The only difference between
 *   send() and write() is the presence of flags. With zero flags parameter,
 *   send() is equivalent to write(). Also, send(sockfd,buf,len,flags) is
 *   equivalent to sendto(sockfd,buf,len,flags,NULL,0).
 *
 * Parameters:
 *   sockfd   Socket descriptor of socket
 *   buf      Data to send
 *   len      Length of data to send
 *   flags    Send flags
 *
 * Returned Value:
 *   On success, returns the number of characters sent.  On  error,
 *   -1 is returned, and errno is set appropriately:
 *
 *   EAGAIN or EWOULDBLOCK
 *     The socket is marked non-blocking and the requested operation
 *     would block.
 *   EBADF
 *     An invalid descriptor was specified.
 *   ECONNRESET
 *     Connection reset by peer.
 *   EDESTADDRREQ
 *     The socket is not connection-mode, and no peer address is set.
 *   EFAULT
 *      An invalid user space address was specified for a parameter.
 *   EINTR
 *      A signal occurred before any data was transmitted.
 *   EINVAL
 *      Invalid argument passed.
 *   EISCONN
 *     The connection-mode socket was connected already but a recipient
 *     was specified. (Now either this error is returned, or the recipient
 *     specification is ignored.)
 *   EMSGSIZE
 *     The socket type requires that message be sent atomically, and the
 *     size of the message to be sent made this impossible.
 *   ENOBUFS
 *     The output queue for a network interface was full. This generally
 *     indicates that the interface has stopped sending, but may be
 *     caused by transient congestion.
 *   ENOMEM
 *     No memory available.
 *   ENOTCONN
 *     The socket is not connected, and no target has been given.
 *   ENOTSOCK
 *     The argument s is not a socket.
 *   EOPNOTSUPP
 *     Some bit in the flags argument is inappropriate for the socket
 *     type.
 *   EPIPE
 *     The local end has been shut down on a connection oriented socket.
 *     In this case the process will also receive a SIGPIPE unless
 *     MSG_NOSIGNAL is set.
 *
 * Assumptions:
 *
 ****************************************************************************/

ssize_t send(int sockfd, FAR const void *buf, size_t len, int flags)
{
  FAR struct socket *psock;
  ssize_t ret;

  /* send() is a cancellation point */

  (void)enter_cancellation_point();

  /* Get the underlying socket structure */

  psock = sockfd_socket(sockfd);

  /* And let psock_send do all of the work */

  ret = psock_send(psock, buf, len, flags);
  leave_cancellation_point();
  return ret;
}
