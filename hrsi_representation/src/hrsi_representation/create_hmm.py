#!/usr/bin/env python

import numpy as np
import ghmm as gh
import os


class QtcException(Exception):
    def __init__(self, message):

        # Call the base class constructor with the parameters it
        # needs
        Exception.__init__(self, "QTC Exception: " + message)


class QTCHMM():

    def __init__(self):
        self.hmm = None
    
    def createSequenceSet(self, qtc, symbols):
        return gh.SequenceSet(symbols, qtc)
    
    
    def createCNDTransEmiProb(self, qtc_type='qtcc'):
        """Creates a Conditional Neighbourhood Diagram as a basis for the HMM"""
    
        if qtc_type == 'qtcb':
            state_num = 11
        elif qtc_type == 'qtcc':
            state_num = 83
        elif qtc_type == 'qtcbc':
            state_num = 92
        else:
            raise(QtcException("createCNDTransEmiProb: Unknow qtc type: {!r}".format(qtc_type)))
    
        qtc = []
    
        if qtc_type == 'qtcb':
            for i in xrange(1, 4):
                for j in xrange(1, 4):
                    qtc.append([i-2, j-2])
        elif qtc_type == 'qtcc':
            for i in xrange(1, 4):
                for j in xrange(1, 4):
                    for k in xrange(1, 4):
                        for l in xrange(1, 4):
                            qtc.append([i-2, j-2, k-2, l-2])
        elif qtc_type == 'qtcbc':
            for i in xrange(1, 4):
                for j in xrange(1, 4):
                    qtc.append([i-2, j-2, np.NaN, np.NaN])
            for i in xrange(1, 4):
                for j in xrange(1, 4):
                    for k in xrange(1, 4):
                        for l in xrange(1, 4):
                            qtc.append([i-2, j-2, k-2, l-2])
        else:
            raise(QtcException("createCNDTransEmiProb: Unknow qtc type: {!r}".format(qtc_type)))
    
        qtc = np.array(qtc)
        #np.savetxt('/home/cdondrup/qtc.csv', qtc, delimiter=',', fmt='%1f')
    
        trans = np.zeros((state_num, state_num))
        for i1 in xrange(qtc.shape[0]):
            for i2 in xrange(i1+1, qtc.shape[0]):
                trans[i1+1, i2+1] = np.nanmax(np.absolute(qtc[i1]-qtc[i2])) != 2
                if trans[i1+1, i2+1] == 1:
                    for j1 in xrange(qtc.shape[1]-1):
                        for j2 in xrange(j1+1, qtc.shape[1]):
                            if sum(np.absolute(qtc[i1, [j1, j2]])) == 1 \
                                    and sum(np.absolute(qtc[i2, [j1, j2]])) == 1:
                                if np.nanmax(np.absolute(qtc[i1, [j1, j2]]-qtc[i2, [j1, j2]])) > 0 \
                                        and sum(qtc[i1, [j1, j2]]-qtc[i2, [j1,j2]]) != 1:
                                    trans[i1+1, i2+1] = 5
                                    break
                        if trans[i1+1, i2+1] != 1:
                            break
                trans[i2+1, i1+1] = trans[i1+1, i2+1]
    
        trans[trans != 1] = 0
        #np.savetxt('/home/cdondrup/trans.csv', np.rint(trans).astype(int), delimiter=',', fmt='%i')
        trans[trans == 0] = 0.00001
        trans[0] = 1
        trans[:, 0] = 0
        trans[:, -1] = 1
        trans[0, -1] = 0
        trans[-1] = 0
        trans += np.dot(np.eye(state_num), 0.00001)
        trans[0, 0] = 0
    
        trans = trans / trans.sum(axis=1).reshape(-1, 1)
        #np.savetxt('/home/cdondrup/trans.csv', trans, delimiter=',')
    
        emi = np.eye(state_num)
        emi[emi == 0] = 0.0001
    
        return trans, emi
    
    
    def qtc2state(self, qtc):
        """Transforms a qtc state to a number"""
    
        state_rep = []
        for idx, element in enumerate(qtc):
#            val_qtc = validateQtcSequences(element)
            d = element.shape[1]
            mult = 3**np.arange(d-1, -1, -1)
            state_num = np.append(
                0,
                ((element + 1)*np.tile(mult, (element.shape[0], 1))).sum(axis=1) + 1
            )
            state_num = np.append(state_num, 82)
            state_char = ''
            for n in state_num:
                state_char += chr(int(n)+32)
            state_rep.append(state_num.tolist())
    
        return state_rep
    
#    Is now validated by qsr_lib
#    def validateQtcSequences(self, qtc):
#        """Removes illegal state transition by inserting necessary intermediate states"""
#    
#        newqtc = qtc[0].copy()
#        j = 1
#        for i in xrange(1, len(qtc)):
#            checksum = np.nonzero(qtc[i-1]+qtc[i] == 0)
#            intermediate = qtc[i].copy()
#            if checksum[0].size > 0:
#                for idx in checksum[0]:
#                    if np.absolute(qtc[i-1][idx]) + np.absolute(qtc[i][idx]) > 0:
#                        intermediate[idx] = 0
#                if np.any(intermediate != qtc[i]):
#                    newqtc = np.append(newqtc, intermediate)
#                    j += 1
#            newqtc = np.append(newqtc, qtc[i])
#            j += 1
#    
#        return newqtc.reshape(-1, 4)
    
    
    def generateAlphabet(self, num_symbols):
        return gh.IntegerRange(0, num_symbols)
    
    
    def trainHMM(self, seq, trans, emi, qtc_type='qtcc'):
        """Uses the given parameters to train a multinominal HMM to represent the given seqences"""
    
        if qtc_type == 'qtcb':
            state_num = 11
        elif qtc_type == 'qtcc':
            state_num = 83
        elif qtc_type == 'qtcbc':
            state_num = 92
        else:
            raise(QtcException("trainHMM: Unknow qtc type: {!r}".format(qtc_type)))
    
        print 'Generating HMM:'
        print '\tCreating symbols...'
        symbols = self.generateAlphabet(state_num)
        startprob = np.zeros((state_num))
        startprob[0] = 1
        print '\t\t', symbols
        print '\tCreating HMM...'
        qtc_hmm = gh.HMMFromMatrices(
            symbols,
            gh.DiscreteDistribution(symbols),
            trans.tolist(),
            emi.tolist(),
            startprob.tolist()
        )
        print '\tTraining...'
        qtc_hmm.baumWelch(self.createSequenceSet(seq, symbols))
    
        return qtc_hmm
    
    
    def createHMM(self, qtc_seq, qtc_type='qtcc'):
        """Create and trains a HMM to represent the given qtc sequences"""
    
        try:
            qtc_state_seq = self.qtc2state(qtc_seq)
            trans, emi = self.createCNDTransEmiProb(qtc_type)
            qtchmm = self.trainHMM(qtc_state_seq, trans, emi, qtc_type)
            print '...done'
            self.hmm = qtchmm
            return qtchmm
        except QtcException as e:
            print e.message
            return None
    
    
#    def createTestSequence(self, seq_path, qtc_type='qtcc'):
#        if qtc_type is 'qtcb':
#            return createSequenceSet(qtc2state(readQtcFiles(seq_path)), generateAlphabet(11))
#        elif qtc_type is 'qtcc':
#            return createSequenceSet(qtc2state(readQtcFiles(seq_path)), generateAlphabet(83))
#        elif qtc_type is 'qtcbc':
#            return createSequenceSet(qtc2state(readQtcFiles(seq_path)), generateAlphabet(92))
#        else:
#            raise(QtcException("Unknow qtc type: {!r}".format(qtc_type)))
