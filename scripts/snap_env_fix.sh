# shellcheck shell=bash
# Source this file (". scripts/snap_env_fix.sh") to undo the environment that
# VS Code installed as a snap leaks into its terminals and tasks.
# Without it, GUI programs started from VS Code (gz sim -g, QGroundControl)
# load libraries from /snap/core20 and die at once with
#   "symbol lookup error: /snap/core20/.../libpthread.so.0 ..."
# Harmless when VS Code is not a snap (nothing to undo).

if [[ -n "${SNAP:-}" || -n "${GTK_PATH:-}" ]]; then
  for _v in GTK_PATH GTK_EXE_PREFIX GTK_IM_MODULE_FILE GDK_PIXBUF_MODULE_FILE \
            GDK_PIXBUF_MODULEDIR GIO_MODULE_DIR GSETTINGS_SCHEMA_DIR LOCPATH \
            GIO_LAUNCHED_DESKTOP_FILE; do
    unset "$_v"
  done
  # VS Code keeps the original values in *_VSCODE_SNAP_ORIG
  [[ -n "${XDG_DATA_DIRS_VSCODE_SNAP_ORIG:-}" ]] && export XDG_DATA_DIRS="$XDG_DATA_DIRS_VSCODE_SNAP_ORIG"
  [[ -n "${XDG_CONFIG_DIRS_VSCODE_SNAP_ORIG:-}" ]] && export XDG_CONFIG_DIRS="$XDG_CONFIG_DIRS_VSCODE_SNAP_ORIG"
  for _v in $(compgen -v SNAP); do unset "$_v"; done
  unset _v
fi
