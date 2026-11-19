"""Shared, immediately flushed console banners for lifecycle transitions."""


def banner(source, stage, message, color="cyan"):
    colors = {"cyan": "\033[1;36m", "green": "\033[1;32m",
              "yellow": "\033[1;33m", "red": "\033[1;31m"}
    border = "################" if color in ("yellow", "red") else ">>>>>>>>>>>>>>>>"
    print("{}{} [{}] {}：{} {}\033[0m".format(
        colors[color], border, source, stage, message, border), flush=True)
