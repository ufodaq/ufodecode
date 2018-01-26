project('libufodecode', 'c')

version = '0.3'
so_version = '0'

cc = meson.get_compiler('c')

have_sse = cc.has_argument('-msse') and cc.has_argument('-msse2')
 
conf = configuration_data()

conf.set('DEBUG', get_option('buildtype') == 'debug')
conf.set('HAVE_SSE', have_sse)
conf.set('IPECAMERA_WIDTH', get_option('ipecamera_width'))

configure_file(
    input: 'config.h.meson.in',
    output: 'config.h',
    configuration: conf
)

lib = shared_library('ufodecode',
    'src/ufodecode.c',
    version: version,
    soversion: so_version,
    install: true
)

install_headers('src/ufodecode.h')

ipedec = executable('ipedec',
    [ 'test/ipedec.c',
      'test/timer.c' ],
    link_with: lib,
    include_directories: include_directories('src'),
    install: true
)

pkg = import('pkgconfig')

pkg.generate(
    libraries: [lib],
    version: version,
    name: 'libufodecode',
    description: 'Decoding routines for the UFO camera'
)
