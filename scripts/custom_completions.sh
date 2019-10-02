#!/bin/bash

# Autocomplete world files in $MOBILE_SIM_HOME/worlds
_mobile_worlds_completion()
{
	local cur=${COMP_WORDS[COMP_CWORD]}
	COMPREPLY=( $(compgen -W "$(ls $MOBILE_SIM_HOME/worlds)" -- $cur) )
}

# Autocomplete map files in $MOBILE_SIM_HOME/map_info
_mobile_map_info_completion()
{
	local cur=${COMP_WORDS[COMP_CWORD]}
	COMPREPLY=( $(compgen -W "$(ls $MOBILE_SIM_HOME/map_info)" -- $cur) )
}

# Autocomplete rosie agents in $ROSIE_HOME/test-agents
_rosie_agents_completion()
{
	local cur=${COMP_WORDS[COMP_CWORD]}
	COMPREPLY=( $(compgen -W "$(ls $ROSIE_HOME/test-agents)" -- $cur) )
}

