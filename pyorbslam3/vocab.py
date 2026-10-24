"""ORB vocabulary download helper.

The 139 MB ``ORBvoc.txt`` is not shipped in the wheel. ``vocabulary_path()``
returns a local copy, downloading and extracting it once into a cache
directory (``$PYORBSLAM3_CACHE`` or ``~/.cache/pyorbslam3``). If the
``PYORBSLAM3_VOCAB`` environment variable points at an existing file, that file
is used instead.
"""

from __future__ import annotations

import hashlib
import os
import sys
import tarfile
import tempfile
import urllib.request

VOCAB_URL = "https://github.com/UZ-SLAMLab/ORB_SLAM3/raw/master/Vocabulary/ORBvoc.txt.tar.gz"
VOCAB_SHA1 = "ad82cc2a05da116bf211192e4748024a19b7606e"  # of the .tar.gz


def _cache_dir() -> str:
    return os.environ.get("PYORBSLAM3_CACHE") or os.path.join(
        os.environ.get("XDG_CACHE_HOME", os.path.expanduser("~/.cache")), "pyorbslam3"
    )


def vocabulary_path(download: bool = True, quiet: bool = False) -> str:
    """Return the path to ORBvoc.txt, fetching it on first use.

    Raises FileNotFoundError if the file is missing and ``download`` is False.
    """
    env = os.environ.get("PYORBSLAM3_VOCAB")
    if env and os.path.isfile(env):
        return env
    target = os.path.join(_cache_dir(), "ORBvoc.txt")
    if os.path.isfile(target):
        return target
    if not download:
        raise FileNotFoundError(f"ORB vocabulary not found at {target}")

    os.makedirs(os.path.dirname(target), exist_ok=True)
    if not quiet:
        print(f"[pyorbslam3] downloading ORB vocabulary (~40 MB) to {target} ...", file=sys.stderr)
    fd, tmp = tempfile.mkstemp(suffix=".tar.gz", dir=os.path.dirname(target))
    os.close(fd)
    try:
        urllib.request.urlretrieve(VOCAB_URL, tmp)
        sha1 = hashlib.sha1()
        with open(tmp, "rb") as f:
            for chunk in iter(lambda: f.read(1 << 20), b""):
                sha1.update(chunk)
        if sha1.hexdigest() != VOCAB_SHA1:
            raise RuntimeError(
                f"vocabulary archive checksum mismatch ({sha1.hexdigest()} != {VOCAB_SHA1})"
            )
        with tarfile.open(tmp, "r:gz") as tar:
            member = next(m for m in tar.getmembers() if m.name.endswith("ORBvoc.txt"))
            with tar.extractfile(member) as src, open(target + ".part", "wb") as dst:
                while True:
                    buf = src.read(1 << 20)
                    if not buf:
                        break
                    dst.write(buf)
        os.replace(target + ".part", target)
    finally:
        if os.path.exists(tmp):
            os.remove(tmp)
    return target
