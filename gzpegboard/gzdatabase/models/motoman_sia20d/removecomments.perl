#!/usr/bin/perl -wp

# remove C style comments

BEGIN { undef $/; }

s|/[*].*?[*]/||sg;
