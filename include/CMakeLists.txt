set(hdrs arm.h)


set(install_hdrs ${hdrs})

install(FILES ${install_hdrs} DESTINATION ${INC_INSTALL_DIR}
    COMPONENT library)

