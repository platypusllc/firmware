#!/bin/sh
#
# Generates a local server with the same settings as github uses.
#

jekyll --pygments --no-lsi --safe --base-url="`pwd`\_site"
