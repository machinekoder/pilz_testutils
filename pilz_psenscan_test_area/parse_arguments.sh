#!/bin/bash

print_usage()
{
  echo "Usage:"
  echo "       $TEST_AREA_SCRIPTS_PATH/automatic_acceptance_test_psen_scan.sh TEST_NAME [OPTIONS]"
  echo ""
  echo "       TEST_NAME"
  echo "              name of test to be executed"
  echo ""
  echo "OPTIONS:"
  echo "       -h, --help"
  echo "              print this help message"
  echo "       --repo REPO"
  echo "              specify which repository should be cloned"
  echo "       --target-branch TARGET_BRANCH"
  echo "              specify which branch, tag or commit should be checked out"
  echo "       --endless"
  echo "              set test to endless mode"
}

PARSE_ARGUMENTS_SUCCESSFUL=1

POSITIONAL=()
while [[ $# -gt 0 ]]
do
key="$1"

case $key in
    -h|--help)
    print_usage
    PARSE_ARGUMENTS_SUCCESSFUL=1
    ;;
    --repo)
    [[ -z $2 ]] || export REPO_NAME="$2"
    shift # past argument
    shift # past value
    ;;
    --target-branch)
    [[ -z $2 ]] || export TARGET_BRANCH="$2"
    shift # past argument
    shift # past value
    ;;
    --endless)
    export ENDLESS=1
    shift # past argument
    ;;
    *)    # unknown option
    POSITIONAL+=("$1") # save it in an array for later
    shift # past argument
    ;;
esac
done

set -- "${POSITIONAL[@]}" # restore positional parameters

[[ $# == 0 ]] && { print_usage; PARSE_ARGUMENTS_SUCCESSFUL=0; }
[[ $# > 1 ]] && { echo "Too many arguments!"; print_usage; PARSE_ARGUMENTS_SUCCESSFUL=0; }

export TEST_NAME="$1"
