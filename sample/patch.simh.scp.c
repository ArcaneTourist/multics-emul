--- scp.c.orig	2013-03-16 13:40:50.000000000 -0500
+++ scp.c	2013-03-16 14:00:30.000000000 -0500
@@ -23,6 +23,10 @@
    used in advertising or otherwise to promote the sale, use or other dealings
    in this Software without prior written authorization from Robert M Supnik.
 
+   10-Dec-12    MM      Pass rptr to fprint_sym to allow output to be tailored
+                        to the specific register such as displaying a condition
+                        register as flag names.  Allow quoted sequences in command
+                        input as a single token.
    20-Mar-12    MP      Fixes to "SHOW <x> SHOW" commands
    06-Jan-12    JDB     Fixed "SHOW DEVICE" with only one enabled unit (Dave Bryan)  
    25-Sep-11    MP      Added the ability for a simulator built with 
@@ -4529,7 +4533,10 @@
 if ((rptr->flags & REG_VMAD) && sim_vm_fprint_addr)
     sim_vm_fprint_addr (ofile, sim_dflt_dev, (t_addr) val);
 else if (!(rptr->flags & REG_VMIO) ||
-    (fprint_sym (ofile, rdx, &val, NULL, sim_switches | SIM_SW_REG) > 0)) {
+    // Pass the rptr to fprint_sym instead of a NULL uptr.
+    // Otherwise, fprint_sym has no way to provide output tailored to
+    // a specifc register (e.g. displaying a conditions register as flags).
+    (fprint_sym (ofile, rdx, &val, (void*) rptr, sim_switches | SIM_SW_REG) > 0)) {
         fprint_val (ofile, val, rdx, rptr->width, rptr->flags & REG_FMT);
         if (rptr->fields) {
             fprintf (ofile, "\t");
@@ -5048,7 +5055,13 @@
 
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
