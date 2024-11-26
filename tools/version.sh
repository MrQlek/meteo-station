#!/bin/bash


commit=$(git rev-parse --short HEAD)
version=v-$commit
git diff-index --quiet HEAD -- || version=$version+ ;
echo $version-v