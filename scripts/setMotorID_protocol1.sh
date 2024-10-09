#!/bin/sh

isuint_Equi() { [[ $1 == +([0-9]) ]] ;}

id=$1
newid=$2

id=20

echo "number of vars = $# "  
echo "Var 0 = $0 "  
echo "Var 1 = $1 "  
echo "Var 2 = $2 "  

if [ $# != 2 ] ; then
    echo "Error: number of vars != 2"
    exit 1
fi

read -sp "You want to change the motor's id from $id to $newid. Are you sure? [y/n]" -n 1 -r
echo    # (optional) move to a new line

if [[ $REPLY =~ ^[Yy]$ ]];then
    echo "Changing the id..."

    # Deactivate filter
echo $((0xA + 1))
    #canid=$id
    #canid=$((0x140))
    canid=$((0x140 + id))
    echo "can id = $canid"

    bc -l

    # Set id

    # Reset?

    # Reactivate filter
else
    echo "Exiting without doing anything"
    exit 0
fi