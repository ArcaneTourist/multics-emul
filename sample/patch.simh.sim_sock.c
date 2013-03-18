--- ../../simh-orig/sim_sock.c	2012-12-06 21:48:24.000000000 -0600
+++ ./sim_sock.c	2012-12-06 22:05:48.000000000 -0600
@@ -154,6 +154,13 @@
 name.sin_port = htons ((unsigned short) port);          /* insert port */
 name.sin_addr.s_addr = htonl (INADDR_ANY);              /* insert addr */
 
+// Don't fail because of a prior SIMH run finished recently
+// enough to still have the socket in TIME_WAIT.
+int one = 1;
+if (setsockopt(newsock, SOL_SOCKET, SO_REUSEADDR, &one, sizeof(one))) {
+    printf("Warning: cannot set socket reuse-addr option\n");
+}
+
 sta = bind (newsock, (struct sockaddr *) &name, sizeof (name));
 if (sta == SOCKET_ERROR)                                /* bind error? */
     return sim_err_sock (newsock, "bind", 1);
