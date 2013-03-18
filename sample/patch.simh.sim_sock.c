--- sim_sock.c.orig	2013-03-16 13:40:51.000000000 -0500
+++ sim_sock.c	2013-03-16 14:09:36.000000000 -0500
@@ -23,6 +23,7 @@
    used in advertising or otherwise to promote the sale, use or other dealings
    in this Software without prior written authorization from Robert M Supnik.
 
+   08-Dec-12    MM      Use SO_REUSEADDR, avoiding TIME_WAIT.
    15-Oct-12    MP      Added definitions needed to detect possible tcp 
                         connect failures
    25-Sep-12    MP      Reworked for RFC3493 interfaces supporting IPv6 and IPv4
@@ -721,6 +722,12 @@
     p_freeaddrinfo(result);
     return newsock;
     }
+// Don't fail because of a prior SIMH run finished recently
+// enough to still have the socket in TIME_WAIT.
+int one = 1;
+if (setsockopt(newsock, SOL_SOCKET, SO_REUSEADDR, &one, sizeof(one))) {
+    printf("Warning: cannot set socket reuse-addr option\n");
+}
 #ifdef IPV6_V6ONLY
 if (preferred->ai_family == AF_INET6) {
     int off = FALSE;
