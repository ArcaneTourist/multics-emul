--- ../../myproj/simh-orig/scp.c	2012-12-06 21:48:23.000000000 -0600
+++ ./scp.c	2012-12-06 22:56:23.000000000 -0600
@@ -3948,7 +3948,10 @@
 if ((rptr->flags & REG_VMAD) && sim_vm_fprint_addr)
     sim_vm_fprint_addr (ofile, sim_dflt_dev, (t_addr) val);
 else if (!(rptr->flags & REG_VMIO) ||
-    (fprint_sym (ofile, rdx, &val, NULL, sim_switches | SIM_SW_REG) > 0))
+    // Pass the rptr to fprint_sym instead of a NULL uptr.
+    // Otherwise, fprint_sym has no way to provide output tailored to
+    // a specifc register (e.g. displaying a conditions register as flags).
+    (fprint_sym (ofile, rdx, &val, (void*) rptr, sim_switches | SIM_SW_REG) > 0))
         fprint_val (ofile, val, rdx, rptr->width, rptr->flags & REG_FMT);
 if (flag & EX_I)
     fprintf (ofile, "\t");
@@ -4460,7 +4463,13 @@
 
 char *get_glyph_gen (char *iptr, char *optr, char mchar, t_bool uc)
 {
-while ((isspace (*iptr) == 0) && (*iptr != 0) && (*iptr != mchar)) {
+int quoting = 0;
+while ((quoting || isspace (*iptr) == 0) && (*iptr != 0) && (*iptr != mchar)) {
+    if (*iptr == '"') {
+        quoting = ! quoting;
+        iptr ++;
+        continue;
+    }
     if (islower (*iptr) && uc)
         *optr = toupper (*iptr);
     else *optr = *iptr;
