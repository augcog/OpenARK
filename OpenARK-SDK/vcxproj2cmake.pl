#!/usr/bin/env perl

use strict;
use warnings;
use utf8;
use feature 'say';
use Readonly;

use autobox::Core;
use Try::Tiny;
use XML::TreePP;
use XML::TreePP::XMLPath;
use Text::Xslate;
use Data::Dumper;
use FindBin;

my $filepath    = $ARGV[0]
    || die "Usage: $FindBin::Script <vcxproj path> <target configuration>";
my $target_conf = $ARGV[1]
    || die "Usage: $FindBin::Script <vcxproj path> <target configuration>";

my $filters_path = $filepath . '.filters';


my $vcxproj_xml  = XML::TreePP->new()->parsefile($filepath);
my $vcxproj_filters_xml  = XML::TreePP->new()->parsefile($filters_path);
my $target = $vcxproj_xml->{Project}->{PropertyGroup}->[0]->{RootNamespace};
my $target_prop = get_target_property($target_conf,
                                      get_propertys($vcxproj_xml));
my $filenode_ref = get_file_node($vcxproj_xml);
my $item_def_ref = get_item_def_group($vcxproj_xml,
                                      $target_prop->{-Condition});
my %filters = get_filters($vcxproj_filters_xml);

make($filenode_ref, $item_def_ref, %filters);

sub get_filters {
    my ($vcxproj_filters_xml) = @_;
    my %filters = ();

    my @itemgroups = @{$vcxproj_filters_xml->{Project}->{ItemGroup}};

    # first itemgroup is list of filters
    for my $filter (@{$itemgroups[0]->{Filter}}) {
        my $filtername = $filter->{-Include};
        my @files = ();
        @{$filters{$filtername}} = @files;
    }

    # second itemgroup contains sorce files filter information
    for my $clcompile (@{$itemgroups[1]->{ClCompile}}) {
        my $filename = $clcompile->{-Include};
        my $filter = $clcompile->{Filter};
        push @{$filters{$filter}}, $filename;
    }

    # third itemgroup contains header files filter information
    for my $clinclude (@{$itemgroups[2]->{ClInclude}}) {
        my $filename = $clinclude->{-Include};
        my $filter = $clinclude->{Filter};
        push @{$filters{$filter}}, $filename;
    }

    # fourth itemgroup contains filter information for files that should not compiled (like readme)

    # fifth itemgroup contains recources filter

    # remove empty arrays
    my @toDelete;
    for my $filter (keys %filters) {
        my @filterfiles = @{$filters{$filter}};
        if (scalar @filterfiles == 0) {
            push @toDelete, $filter;
        }
    }
    #delete @filters{@toDelete};

    return %filters;
}

sub get_target_property {
    my ($target_conf, @property_groups) = @_;
    my $target_prop;
    for my $property (@property_groups) {
        if ($property->{-Condition} =~ m{$target_conf}) {
            $target_prop = $property;
            last;
        }
    }

    return $target_prop;
}

sub get_item_def_group {
    my ($vcxproj_xml, $cond) = @_;
    my $tppx = new XML::TreePP::XMLPath;

    # my $xpath = '/Project/ItemDefinitionGroup[@Condition="'.$cond.'"]';
    # warn $xpath;
    # my $item_def_ref = $tppx->filterXMLDoc($vcxproj_xml, $xpath);
    my $item_def_ref;
    for my $item_def ($tppx->filterXMLDoc($vcxproj_xml,
                                          '/Project/ItemDefinitionGroup')) {
        if ($item_def->{-Condition} eq $cond) {
            $item_def_ref = $item_def;
            last;
        }
    }

    return $item_def_ref;
}

sub get_propertys {
    my ($vcxproj_xml) = @_;
    my $property_node_ref;

    my $tppx = new XML::TreePP::XMLPath;
    my @property_groups
        = $tppx->filterXMLDoc($vcxproj_xml,
                              '/Project/PropertyGroup[@Label="Configuration"]');

    return @property_groups;
}

sub get_file_node {
    my ($vcxproj_xml) = @_;
    my @itemgroups = @{$vcxproj_xml->{Project}->{ItemGroup}};
    my $filenode_ref;
    for my $itemgroup (@itemgroups) {
        next if (defined $itemgroup->{-Label}
                     && $itemgroup->{-Label} eq 'ProjectConfigurations');

        for my $nodename (keys %{$itemgroup}) {
            my @nodes;

            if (ref($itemgroup->{$nodename}) eq 'HASH') {
                push @nodes, $itemgroup->{$nodename};
            } else {
                @nodes = @{$itemgroup->{$nodename}};
            }

            for my $node (@nodes) {
                unless (defined $node->{ExcludedFromBuild}
                            && $node->{ExcludedFromBuild}->{-Condition} eq $target_prop->{-Condition}) {
                    push @{$filenode_ref->{$nodename}}, $node->{-Include};
                }
            }
        }
    }

    return $filenode_ref;
}

sub render_find_packages {
    my ($tx, @srcs) = @_;
    my %find_packages_supported_by_cmake = (
        # include_pattern => package_name

        #'' => 'ALSA',
        #'' => 'Armadillo',
        #'' => 'ASPELL',
        #'' => 'AVIFile',
        #'' => 'BISON',
        #'' => 'BLAS',
        'boost/' => 'Boost',
        #'' => 'Bullet',
        #'' => 'BZip2',
        #'' => 'CABLE',
        #'' => 'Coin3D',
        #'' => 'CUDA',
        #'' => 'Cups',
        #'' => 'CURL',
        #'' => 'Curses',
        #'' => 'CVS',
        #'' => 'CxxTest',
        #'' => 'Cygwin',
        #'' => 'Dart',
        #'' => 'DCMTK',
        #'' => 'DevIL',
        #'' => 'Doxygen',
        #'' => 'EXPAT',
        #'' => 'FLEX',
        #'' => 'FLTK',
        #'' => 'FLTK2',
        #'' => 'Freetype',
        #'' => 'GCCXML',
        #'' => 'GDAL',
        #'' => 'Gettext',
        #'' => 'GIF',
        #'' => 'Git',
        #'' => 'GLU',
        #'' => 'GLUT',
        #'' => 'Gnuplot',
        #'' => 'GnuTLS',
        #'' => 'GTest',
        #'' => 'GTK',
        #'' => 'GTK2',
        #'' => 'HDF5',
        #'' => 'HSPELL',
        #'' => 'HTMLHelp',
        #'' => 'ImageMagick',
        #'' => 'ITK',
        #'' => 'Jasper',
        #'' => 'Java',
        #'' => 'JNI',
        #'' => 'JPEG',
        #'' => 'KDE3',
        #'' => 'KDE4',
        #'' => 'LAPACK',
        #'' => 'LATEX',
        #'' => 'LibArchive',
        #'' => 'LibLZMA',
        #'' => 'LibXml2',
        #'' => 'LibXslt',
        #'' => 'Lua50',
        #'' => 'Lua51',
        #'' => 'Matlab',
        #'' => 'MFC',
        #'' => 'Motif',
        #'' => 'MPEG',
        #'' => 'MPEG2',
        #'' => 'MPI',
        #'' => 'OpenAL',
        #'' => 'OpenGL',
        #'' => 'OpenMP',
        #'' => 'OpenSceneGraph',
        'openssl/' => 'OpenSSL',
        #'' => 'OpenThreads',
        #'' => 'osg',
        #'' => 'osgAnimation',
        #'' => 'osgDB',
        #'' => 'osgFX',
        #'' => 'osgGA',
        #'' => 'osgIntrospection',
        #'' => 'osgManipulator',
        #'' => 'osgParticle',
        #'' => 'osgPresentation',
        #'' => 'osgProducer',
        #'' => 'osgQt',
        #'' => 'osgShadow',
        #'' => 'osgSim',
        #'' => 'osgTerrain',
        #'' => 'osgText',
        #'' => 'osgUtil',
        #'' => 'osgViewer',
        #'' => 'osgVolume',
        #'' => 'osgWidget',
        #'' => 'osg_functions',
        #'' => 'PackageHandleStandardArgs',
        #'' => 'PackageMessage',
        #'' => 'Perl',
        #'' => 'PerlLibs',
        #'' => 'PHP4',
        #'' => 'PhysFS',
        #'' => 'Pike',
        #'' => 'PkgConfig',
        #'' => 'PNG',
        #'' => 'PostgreSQL',
        #'' => 'Producer',
        #'' => 'Protobuf',
        #'' => 'PythonInterp',
        #'' => 'PythonLibs',
        #'' => 'Qt',
        #'' => 'Qt3',
        #'' => 'Qt4',
        #'' => 'QuickTime',
        #'' => 'RTI',
        #'' => 'Ruby',
        'SDL/' => 'SDL',
        #'' => 'SDL_image',
        #'' => 'SDL_mixer',
        #'' => 'SDL_net',
        #'' => 'SDL_sound',
        #'' => 'SDL_ttf',
        #'' => 'SelfPackers',
        #'' => 'Squish',
        #'' => 'Subversion',
        #'' => 'SWIG',
        #'' => 'TCL',
        #'' => 'Tclsh',
        #'' => 'TclStub',
        #'' => 'Threads',
        #'' => 'TIFF',
        #'' => 'UnixCommands',
        #'' => 'VTK',
        #'' => 'Wget',
        #'' => 'Wish',
        #'' => 'wxWidgets',
        #'' => 'wxWindows',
        #'' => 'X11',
		'xercesc' => 'XercesC',
        #'' => 'XMLRPC',
        'zlib.h' => 'ZLIB',
    );
    my $rendered_find_packages = '';

    # find all libs that used
    my @libs_to_find_patterns = keys(%find_packages_supported_by_cmake);
    my @libs;
    for my $file (@srcs) {
        # find libs that the file uses
        open my $info, $file or die "Could not open $file: $!";
        while( my $line = <$info>)  {
            if ($line =~ /#\s*include/) {
                my @lib_pattern_indexes_to_delete;

                for my $lib_match (@libs_to_find_patterns) {
                    if ($line =~ m/^\s*#\s*include\s*<$lib_match/) {
                        my $lib = $find_packages_supported_by_cmake{$lib_match};
                        push @libs, $lib;

                        my $lib_index = 0;
                        $lib_index++ until $libs_to_find_patterns[$lib_index] eq $lib_match;
                        push @lib_pattern_indexes_to_delete, $lib_index;
                    }
                }

                for my $index (@lib_pattern_indexes_to_delete) {
                    splice(@libs_to_find_patterns, $index, 1);
                }
            }
        }
        close $info;
    }

    for my $lib (@libs) {
        my %vars = (
            package => $lib,
            package_upcase => uc $lib,
            );
        warn Dumper(%vars);

        try {
            my $render = $tx->render( 'CMakeFindPackage.tx', \%vars );
            $rendered_find_packages = $rendered_find_packages . $render;
        } catch {
            print "caught error: $_\n";
        };
    }

    return Text::Xslate::Util::mark_raw($rendered_find_packages);
}

sub render_source_groups {
    my ($tx, %filters) = @_;

    my $rendered_source_groups = '';
    for my $filter (keys %filters) {
        my @files = @{$filters{$filter}};
        $filter =~ s/\\/\\\\/g;
        my %vars = (
            flter => $filter,
            files => +(join "\n\t", @files),
        );
        warn Dumper(%vars);

        try {
            my $render = $tx->render( 'CMakeSourceGroup.tx', \%vars );
            $rendered_source_groups = $rendered_source_groups . $render;
        } catch {
            print "caught error: $_\n";
        };
    }

    return Text::Xslate::Util::mark_raw($rendered_source_groups);
}

sub make {
    my ($filenode_ref, $item_def_ref, %filters) = @_;

    my @srcs    = @{$filenode_ref->{ClCompile}};
    my @headers = @{$filenode_ref->{ClInclude}};
    my $add_dir = $item_def_ref->{ClCompile}->{AdditionalIncludeDirectories};
    $add_dir =~ s{\\}{/}g;
    my @includes =
        grep { $_ ne '%(AdditionalIncludeDirectories)' }
            split ';', $add_dir;

    my $def = $item_def_ref->{ClCompile}->{PreprocessorDefinitions};
    # add prefix -D. example> -DSHP
    my @defs =
        map { "-D$_"; }
            grep { $_ ne '%(PreprocessorDefinitions)' }
                split ';', $def;

    my $add_dep = $item_def_ref->{Link}->{AdditionalDependencies};
    $add_dep =~ s{\\}{/}g;
    my @deps =
        grep { $_ ne '%(AdditionalDependencies)' }
            split ';', $add_dep;

    warn Dumper(@includes);

    my $tx = Text::Xslate->new(
        syntax    => 'TTerse',
        path      => [ '.', '.' ],
    );

    my $rendered_find_packages = render_find_packages($tx, @headers, @srcs);
    my $rendered_source_groups = render_source_groups($tx, %filters);

    my %vars = (
        type            => $target_prop->{ConfigurationType},
        charset         => $target_prop->{CharacterSet},
        target          => $target,
        def             => +(join "\n\t", @defs),
        lib             => +(join "\n\t", @deps),
        include         => +(join "\n\t", @includes),
        src             => +(join "\n\t", @srcs),
        header          => +(join "\n\t", @headers),
        source_groups   => $rendered_source_groups,
        find_packages   => $rendered_find_packages,
    );

    warn Dumper(%vars);

    my $render_text;
    try {
        $render_text = $tx->render( 'CMakeLists.tx', \%vars );
    } catch {
        print "caught error: $_\n";
    };

    open my $cmakelists_file, '>', 'CMakeLists.txt';
    print {$cmakelists_file} $render_text;
}
