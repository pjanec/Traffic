#!/usr/bin/env bash
#
# install-sumo.sh
# ---------------
# Installs the pinned SUMO version into the CURRENT (volatile) environment so that
# golden test data can be regenerated. This is a NETWORK-side operation.
#
# IMPORTANT WORKFLOW NOTES:
#   * The VM is volatile. This pip install does NOT persist. That is fine.
#   * SUMO is NEVER required to run `dotnet test`. Tests compare against committed
#     golden files. SUMO is only needed to (re)generate those goldens.
#   * Therefore this script is run deliberately and rarely, only as the first step
#     of regenerating goldens (see regen-goldens.sh), and always ends in a commit
#     of the produced golden files — never inside the offline test loop.
#
# Safe to run on a completely blank VM: assumes nothing is pre-installed except
# python3 + pip.

set -euo pipefail

# Resolve repo root regardless of where the script is invoked from.
REPO_ROOT="$(git rev-parse --show-toplevel)"
cd "$REPO_ROOT"

# Single source of truth for the version.
if [[ ! -f "$REPO_ROOT/SUMO_VERSION" ]]; then
  echo "ERROR: SUMO_VERSION file not found at repo root ($REPO_ROOT)." >&2
  exit 1
fi
# shellcheck disable=SC1091
source "$REPO_ROOT/SUMO_VERSION"

if [[ -z "${SUMO_VERSION:-}" ]]; then
  echo "ERROR: SUMO_VERSION is empty. Set it in $REPO_ROOT/SUMO_VERSION." >&2
  exit 1
fi

echo "==> Installing SUMO (eclipse-sumo==${SUMO_VERSION}) via pip ..."
python3 -m pip install "eclipse-sumo==${SUMO_VERSION}"

# The pip package exposes the `sumo` entry point. Verify it runs and reports the
# expected version so a silent version mismatch fails loudly here, not later.
echo "==> Verifying SUMO install ..."
if ! command -v sumo >/dev/null 2>&1; then
  echo "ERROR: 'sumo' not found on PATH after pip install." >&2
  echo "       The eclipse-sumo wheel should provide it; check pip output above." >&2
  exit 1
fi

INSTALLED_VERSION_LINE="$(sumo --version 2>&1 | head -n 1 || true)"
echo "    sumo --version => ${INSTALLED_VERSION_LINE}"

if ! sumo --version 2>&1 | grep -q "${SUMO_VERSION}"; then
  echo "WARNING: 'sumo --version' output does not contain '${SUMO_VERSION}'." >&2
  echo "         Continuing, but verify this is the intended build." >&2
fi

echo "==> SUMO ${SUMO_VERSION} ready."
