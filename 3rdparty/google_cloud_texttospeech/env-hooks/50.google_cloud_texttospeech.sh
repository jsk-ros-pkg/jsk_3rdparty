# bin/ of this workspace: in the devel space, $(find google_cloud_texttospeech)
# is the source directory, while the text2wave that runs in the virtualenv
# (python3) is generated in <devel>/share/google_cloud_texttospeech/bin
export GOOGLE_CLOUD_TEXTTOSPEECH_BIN_DIR="$CATKIN_ENV_HOOK_WORKSPACE/share/google_cloud_texttospeech/bin"
